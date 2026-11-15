# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026, Alec Dusheck
#
# Worst-case IRAM stack depth of the linked firmware, from the assembly of
# every module it links (ours and the library's).
#
# Each code label is a node. Scanning a node's instructions in order tracks
# the stack balance from its entry: push/pop, inc/dec sp, SDCC's frame moves
# (mov a,sp / add a,#n / mov sp,a and mov sp,_bp). A node's depth is its peak
# balance or, for each lcall/acall, the balance + 2 + the callee's depth, and
# for each jump to another label or fall-through into the next one, the
# balance + that label's depth. Every way into a local label must bring the
# same balance. Recursion, an unknown target, a mismatched balance and an SP
# or _bp write of any other form stop the check (gawk: strtonum).
#
# main is jumped to from reset, so it starts with nothing on the stack. An
# interrupt pushes its return address (2) on top of whatever runs; a low
# priority one (lo) can interrupt main, a high priority one (hi) can also
# interrupt a low priority one. An ISR in both lists is counted at either
# level, but never on top of itself.
#
# -v avail=bytes -v margin=bytes -v lo="_isr ..." -v hi="_isr ..."; prints
# one line, exits 1 when less than margin is left.

function die(msg) { print "error: stack: " msg > "/dev/stderr"; failed = 1; exit 1 }

# A signed byte operand.
function byte(s,  v) { v = strtonum(s); return v > 127 ? v - 256 : v }

# The balance at local label l, wherever it is reached from, is the same.
function meet(l,  k) {
    k = FILENAME SUBSEP node SUBSEP l
    if (k in at && at[k] != b) die("balance " b " at " l " in " node " was " at[k] " elsewhere")
    at[k] = b
}

function edge(to, off) {
    if (node == "") return
    n_e[node]++
    e_to[node, n_e[node]] = to
    e_off[node, n_e[node]] = off
}

function reach(f, from,  i) {
    if (f in depth) return
    if (!(f in defined)) die("no code for " f ", called or jumped to from " from)
    depth[f] = peak[f]
    nodes++
    for (i = 1; i <= n_e[f]; i++)
        reach(e_to[f, i], f)
}

# Longest path by relaxation: loops that keep the balance settle; recursion
# or a loop that grows the stack never does.
function solve(  f, i, d, round, changed) {
    for (changed = 1; changed; round++) {
        if (round > nodes) die("recursion or a stack-growing loop through " changed)
        changed = 0
        for (f in depth)
            for (i = 1; i <= n_e[f]; i++) {
                d = e_off[f, i] + depth[e_to[f, i]]
                if (d > depth[f]) { depth[f] = d; changed = f }
            }
    }
}

{
    line = tolower($0)
    sub(/;.*/, "", line)
}

# A new area ends the previous node; nothing falls through.
line ~ /^[ \t]*\.area/ { node = ""; next }

# A label: a new node or, for a local one (ending in $), a point where the
# fall-through and every jump to it must agree on the balance.
match(line, /^[a-z0-9_.$]+::?/) {
    label = substr(line, 1, RLENGTH)
    sub(/:+$/, "", label)
    line = substr(line, RLENGTH + 1)
    asp = 0
    if (label !~ /\$$/) {
        if (node != "" && fall) edge(label, b)
        node = label
        defined[node] = 1
        b = 0
        bp = ""
    } else if (node != "") {
        if (!fall && (FILENAME, node, label) in at) b = at[FILENAME, node, label]
        meet(label)
    }
    fall = 1
}

node == "" { next }

{
    sub(/^[ \t]+/, "", line)
    op = arg = line
    sub(/[ \t].*/, "", op)
    sub(/^[^ \t]*/, "", arg)
    gsub(/[ \t]/, "", arg)
    if (op == "" || op ~ /^\./) next
    fall = 1
    n = split(arg, o, ",")
    arg = ""
    for (i = 1; i <= n; i++)
        arg = arg (i > 1 ? "," : "") (o[i] ~ /^\(?0x81\)?$/ ? "sp" : o[i])

    # asp: a holds sp + aoff (relative to b).
    if (op == "mov" && arg == "a,sp") { asp = 1; aoff = 0 }
    else if (op == "add" && arg ~ /^a,#/ && asp) aoff += byte(substr(arg, 4))
    else if (arg ~ /^(a|ab|acc)(,|$)/ || op ~ /call$/) asp = 0

    if (op == "push") b++
    else if (op == "pop" && arg == "_bp") { b--; bp = "" }
    else if (op == "pop" && arg != "sp") b--
    else if ((op == "inc" || op == "dec") && arg == "sp") b += op == "inc" ? 1 : -1
    else if (op == "mov" && arg ~ /^(a|r[0-7]),(sp|_bp)$/) ;
    else if (op == "mov" && arg == "_bp,sp") bp = b
    else if (op == "mov" && arg == "_bp,a" && asp) bp = b + aoff
    else if (op == "mov" && arg == "sp,_bp" && bp != "") b = bp
    else if (op == "mov" && arg == "sp,a" && asp) b += aoff
    else if (op == "mov" && arg ~ /^sp,#/ && node == "__sdcc_gsinit_startup") b = 0
    else if (arg ~ /(^|,)(sp|_bp)(,|$)/) die("unmodelled SP or _bp use in " node ": " op " " arg)
    else if (op == "lcall" || op == "acall") edge(arg, b + 2)
    else if (op ~ /^(ljmp|ajmp|sjmp|jmp|jz|jnz|jc|jnc|jb|jnb|jbc|cjne|djnz)$/) {
        t = arg
        sub(/.*,/, "", t)
        if (t ~ /\$$/) meet(t)
        else if (t != "." && t != "@a+dptr") edge(t, b)
        if (node == "__interrupt_vect" && t != "__sdcc_gsinit_startup") isr[t] = 1
        if (op ~ /^(ljmp|ajmp|sjmp|jmp)$/) fall = 0
    } else if (op == "ret" || op == "reti") fall = 0

    if (b > peak[node]) peak[node] = b
}

END {
    if (failed) exit 1
    nl = split(tolower(lo), l, " ")
    nh = split(tolower(hi), h, " ")
    for (t in isr)
        if (index(" " tolower(lo) " " tolower(hi) " ", " " t " ") == 0)
            die(t " is in the vector table but has no priority here")
    reach("_main", "reset")
    for (i = 1; i <= nl; i++) reach(l[i], "lo")
    for (j = 1; j <= nh; j++) reach(h[j], "hi")
    solve()
    worst = m = depth["_main"]
    for (i = 1; i <= nl; i++)
        for (j = 1; j <= nh; j++)
            if (l[i] != h[j] && m + 4 + depth[l[i]] + depth[h[j]] > worst) {
                worst = m + 4 + depth[l[i]] + depth[h[j]]
                wl = l[i]
                wh = h[j]
            }
    printf "stack: worst case %d B of %d, margin %d (main %d, low-prio ISR %s %d, high-prio ISR %s %d)\n",
        worst, avail, margin, m, substr(wl, 2), 2 + depth[wl], substr(wh, 2), 2 + depth[wh]
    if (worst + margin > avail) {
        print "error: stack: worst case " worst " B leaves under " margin " B of the " avail " B from SP to the top of IRAM" > "/dev/stderr"
        exit 1
    }
}
