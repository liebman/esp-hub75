#!/bin/bash
set -e

# Run the HIL suite on the attached board.
#
#   hil/run.sh <width> [options]
#
# The width is required and is never guessed: it describes the *wiring*, not the
# chip (16-bit direct drive vs 8-bit latched), and getting it wrong does not
# fail -- it flashes, boots and passes on a panel that shows nothing. The chip
# is detected from the probe with `probe-rs info` unless --chip overrides it.
#
# Probe IDs are logged per run and never stored: USB-Serial-JTAG gives every
# board its own ID, so a recorded one would bind a run to one board or go stale
# the moment another board is attached (PLAN.md section 10).
#
# Exit status: 0 every run passed, 1 a test or the runner failed, 2 bad usage or
# an ambiguous bench.

usage() {
    cat <<'EOF'
usage: hil/run.sh <width> [options]

  <width>              16 (direct drive) or 8 (latched) -- required

options:
  --chip <chip>        esp32 | esp32s3 | esp32c6 | esp32c5
                       default: detected from the attached probe
  --probe <id>         pin the run to one probe: vid:pid:serial, vid:pid, or a
                       bare serial (matched against the attached probes)
  --probe-index <n>    pick probe [n] from the live `probe-rs list`
  --test <bin> ...     default: all six test binaries
  --mode <mode> ...    default: default full-chain-dma circular-dma
  --features <list>    extra cargo features to add to every run
  --list               resolve and print the matrix, then stop
  --compile-only       `cargo check` only; no probe needed
  -h, --help           this text

examples:
  hil/run.sh 16 --list
  hil/run.sh 8 --test construct refresh
  hil/run.sh 8 --mode full-chain-dma --chip esp32c6
EOF
}

die() {
    echo "run.sh: error: $*" >&2
    exit 2
}

# The driver's own supported set; anything else is a bench mistake, not a
# missing feature.
CHIPS=(esp32 esp32s3 esp32c6 esp32c5)
BINS=(construct reset lifecycle stress async_swap refresh)
MODES=(default full-chain-dma circular-dma)
# `PARL_IO` cannot chain circularly on the C6, and the C5 has no 16-bit
# parallel mode at all (`TxSixteenBits` does not exist for it).
NO_CIRCULAR=(esp32c6)
NO_16BIT=(esp32c5)

contains() {
    local needle="$1" x
    shift
    for x in "$@"; do
        [ "$x" = "$needle" ] && return 0
    done
    return 1
}

join() {
    local IFS=,
    echo "$*"
}

# --- arguments -------------------------------------------------------------

width=""
chip_arg=""
probe_arg=""
probe_index=""
bins=()
modes=()
explicit_bins=false
explicit_modes=false
extra_features=()
list_only=false
compile_only=false

while [ $# -gt 0 ]; do
    case "$1" in
    -h | --help)
        usage
        exit 0
        ;;
    --chip)
        [ $# -ge 2 ] || die "--chip needs a value"
        chip_arg="$2"
        shift 2
        ;;
    --probe)
        [ $# -ge 2 ] || die "--probe needs a value"
        probe_arg="$2"
        shift 2
        ;;
    --probe-index)
        [ $# -ge 2 ] || die "--probe-index needs a value"
        probe_index="$2"
        shift 2
        ;;
    --features)
        [ $# -ge 2 ] || die "--features needs a value"
        extra_features+=("$2")
        shift 2
        ;;
    --test)
        shift
        explicit_bins=true
        while [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; do
            bins+=("$1")
            shift
        done
        ;;
    --mode)
        shift
        explicit_modes=true
        while [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; do
            modes+=("$1")
            shift
        done
        ;;
    --list)
        list_only=true
        shift
        ;;
    --compile-only)
        compile_only=true
        shift
        ;;
    -*)
        die "unknown option '$1' (try --help)"
        ;;
    *)
        [ -z "$width" ] || die "unexpected argument '$1' (the width is the only positional argument)"
        width="$1"
        shift
        ;;
    esac
done

case "$width" in
16 | 8) ;;
"") die "the width is required: 16 (direct drive) or 8 (latched)" ;;
*) die "invalid width '$width': expected 16 or 8" ;;
esac

if [ -n "$chip_arg" ] && ! contains "$chip_arg" "${CHIPS[@]}"; then
    die "invalid chip '$chip_arg': expected one of ${CHIPS[*]}"
fi

# Static combination checks. These need no probe, so they come first and the
# message is about the wiring rather than about whatever is attached.
if [ "$width" = 16 ] && [ -n "$chip_arg" ] && contains "$chip_arg" "${NO_16BIT[@]}"; then
    die "the $chip_arg has no 16-bit parallel mode (PARL_IO has no TxSixteenBits); use width 8"
fi

if [ "$explicit_bins" = true ] && [ ${#bins[@]} -eq 0 ]; then
    die "--test needs at least one binary (${BINS[*]})"
fi
if [ ${#bins[@]} -eq 0 ]; then
    bins=("${BINS[@]}")
fi
for bin in "${bins[@]}"; do
    contains "$bin" "${BINS[@]}" || die "unknown test binary '$bin': expected one of ${BINS[*]}"
done

if [ "$explicit_modes" = true ] && [ ${#modes[@]} -eq 0 ]; then
    die "--mode needs at least one mode (${MODES[*]})"
fi
if [ ${#modes[@]} -eq 0 ]; then
    modes=("${MODES[@]}")
fi
for mode in "${modes[@]}"; do
    contains "$mode" "${MODES[@]}" || die "unknown mode '$mode': expected one of ${MODES[*]}"
done

if [ -n "$probe_arg" ] && [ -n "$probe_index" ]; then
    die "--probe and --probe-index are two ways to say the same thing; pick one"
fi

# The aliases and the per-chip config files are only found from `hil/`.
[ -f .cargo/config.toml ] && [ -f Cargo.toml ] ||
    die "run me from hil/ (cd hil && ./run.sh $width ...)"

# --- probe selection -------------------------------------------------------

# Everything below reads the *live* enumeration, so swapping a board is
# invisible until the next invocation rather than silently reusing an ID.
probe_triples=()
probe_lines=()

list_probes() {
    probe_triples=()
    probe_lines=()
    local line triple
    while IFS= read -r line; do
        case "$line" in
        \[[0-9]*\]:*)
            triple=$(echo "$line" | sed -n 's/.* -- \([^ ]*\) (.*/\1/p')
            if [ -n "$triple" ]; then
                probe_triples+=("$triple")
                probe_lines+=("$line")
            fi
            ;;
        esac
    done < <(probe-rs list 2>/dev/null || true)
}

detect_chip() {
    # Read-only, and no ELF needed: `probe-rs info` alone prints
    # `Detected chip: esp32s3`, which is already the cargo feature name. A probe
    # that another process still holds (the back-to-back case in PLAN.md section
    # 10) comes back empty, so it is worth one retry before giving up.
    local id="$1" tries=0 out=""
    while [ "$tries" -lt 2 ]; do
        out=$(PROBE_RS_NON_INTERACTIVE=true probe-rs info --probe "$id" 2>/dev/null |
            sed -n 's/^Detected chip: *\([^ ]*\).*/\1/p' | head -1)
        if [ -n "$out" ]; then
            break
        fi
        tries=$((tries + 1))
        sleep 2
    done
    echo "$out"
}

describe_probes() {
    local i
    for i in "${!probe_triples[@]}"; do
        printf '  [%d] chip %s -- %s\n' "$i" "$(detect_chip "${probe_triples[$i]}")" "${probe_lines[$i]}"
    done
}

select_selector() {
    local arg="$1" t exact="" candidates=()
    for t in "${probe_triples[@]}"; do
        if [ "$t" = "$arg" ]; then
            exact="$t"
        fi
    done
    if [ -n "$exact" ]; then
        selector="$exact"
        return 0
    fi
    # `vid:pid` (prefix) and a bare serial (suffix) are both accepted, but only
    # when they pick exactly one attached probe: `303a:1001` is every ESP
    # USB-Serial-JTAG probe, so with two boards it is ambiguous. A bare serial is
    # not a valid `--probe` argument for probe-rs itself, which is why it is
    # expanded here instead of being passed through.
    for t in "${probe_triples[@]}"; do
        case "$t" in
        "$arg":* | *":$arg") candidates+=("$t") ;;
        esac
    done
    if [ ${#candidates[@]} -eq 0 ]; then
        die "--probe '$arg' matches no attached probe (try 'probe-rs list')"
    fi
    if [ ${#candidates[@]} -gt 1 ]; then
        die "--probe '$arg' matches ${#candidates[@]} attached probes; use the full vid:pid:serial"
    fi
    selector="${candidates[0]}"
}

selector=""
selector_source=""
list_probes

if [ -n "$probe_arg" ]; then
    select_selector "$probe_arg"
    selector_source="--probe"
elif [ -n "$probe_index" ]; then
    case "$probe_index" in
    *[!0-9]*) die "--probe-index needs a number, got '$probe_index'" ;;
    esac
    if [ "$probe_index" -ge ${#probe_triples[@]} ]; then
        die "--probe-index $probe_index is out of range: ${#probe_triples[@]} probe(s) attached"
    fi
    selector="${probe_triples[$probe_index]}"
    selector_source="--probe-index"
elif [ -n "${PROBE_RS_PROBE:-}" ]; then
    select_selector "$PROBE_RS_PROBE"
    selector_source="PROBE_RS_PROBE"
fi

if [ -z "$selector" ]; then
    if [ ${#probe_triples[@]} -eq 1 ]; then
        selector="${probe_triples[0]}"
        selector_source="the only probe attached"
    elif [ "$compile_only" = false ]; then
        if [ ${#probe_triples[@]} -eq 0 ]; then
            die "no probe found (try 'probe-rs list')"
        fi
        {
            echo "run.sh: ${#probe_triples[@]} probes are attached and a run has to name one:"
            describe_probes
            echo "re-run with --probe-index <n> or --probe <vid:pid:serial>"
        } >&2
        exit 2
    fi
fi

# --- chip ------------------------------------------------------------------

detected=""
if [ -z "$chip_arg" ] || [ "$compile_only" = false ]; then
    # A compile-only run that was told the chip needs no hardware at all, so the
    # mismatch guard (which exists to stop a wrong *flash*) does not apply.
    if [ -n "$selector" ]; then
        detected=$(detect_chip "$selector")
    fi
fi

if [ -n "$chip_arg" ]; then
    chip="$chip_arg"
    if [ -n "$detected" ] && [ "$detected" != "$chip" ]; then
        # Never guess past this: the wrong image is a flash failure at best and
        # a silently wrong board at worst.
        die "the probe reports '$detected' but --chip says '$chip'; refusing to flash one chip's images onto another board"
    fi
    if [ -z "$detected" ] && [ "$compile_only" = false ]; then
        echo "run.sh: warning: could not read a chip from the probe; running with --chip $chip unverified" >&2
    fi
else
    if [ -z "$detected" ]; then
        die "could not determine the chip from the probe; pass --chip (one of ${CHIPS[*]})"
    fi
    chip="$detected"
fi

if ! contains "$chip" "${CHIPS[@]}"; then
    die "the probe reports '$chip', which this suite does not support (${CHIPS[*]})"
fi

if [ "$width" = 16 ] && contains "$chip" "${NO_16BIT[@]}"; then
    die "the $chip has no 16-bit parallel mode (PARL_IO has no TxSixteenBits); use width 8"
fi

skipped_note=""
if contains "$chip" "${NO_CIRCULAR[@]}" && contains circular-dma "${modes[@]}"; then
    if [ "$explicit_modes" = true ]; then
        die "$chip cannot run circular-dma: PARL_IO has no circular chain on the C6"
    fi
    kept=()
    for mode in "${modes[@]}"; do
        if [ "$mode" != circular-dma ]; then
            kept+=("$mode")
        fi
    done
    modes=("${kept[@]}")
    skipped_note="circular-dma on $chip: PARL_IO has no circular chain"
fi

# --- what will run ---------------------------------------------------------

width_note="16-bit direct drive"
test_alias="test-$chip"
check_alias="check-$chip"
if [ "$width" = 8 ]; then
    width_note="8-bit latched"
    test_alias="test-$chip-8"
    check_alias="check-$chip-8"
fi

if [ "$compile_only" = true ]; then
    total=${#modes[@]}
else
    total=$((${#bins[@]} * ${#modes[@]}))
fi

echo "=========================================="
if [ "$compile_only" = true ]; then
    echo "HIL check-only: $chip, $width_note"
else
    echo "HIL run: $chip, $width_note"
fi
if [ "$compile_only" = true ] && [ -n "$chip_arg" ]; then
    # Nothing below opens the probe: the chip was given, so detection was skipped.
    echo "probe  : not touched (compile-only)"
else
    echo "probe  : ${selector:-none needed}${selector_source:+ ($selector_source)}"
fi
echo "width  : $width (alias $test_alias)"
echo "modes  : ${modes[*]}"
echo "bins   : ${bins[*]}"
if [ -n "$skipped_note" ]; then
    echo "skipped: $skipped_note"
fi

if [ "$list_only" = true ]; then
    echo "matrix :"
    if [ "$compile_only" = true ]; then
        for mode in "${modes[@]}"; do
            printf '  %-16s %s\n' "$mode" "$check_alias"
        done
    else
        for bin in "${bins[@]}"; do
            for mode in "${modes[@]}"; do
                printf '  %-12s %s\n' "$bin" "$mode"
            done
        done
    fi
    echo "=========================================="
    echo "--list: nothing was run"
    exit 0
fi
echo "logs   : target/run-logs"
echo "=========================================="

# The runner line in `.cargo/config-<chip>.toml` is `probe-rs run --chip <chip>`
# with no probe and no `--non-interactive`; these two variables are how a chosen
# probe and a non-blocking run reach it (PLAN.md section 10).
export PROBE_RS_NON_INTERACTIVE=true
if [ -n "$selector" ]; then
    export PROBE_RS_PROBE="$selector"
fi

mkdir -p target/run-logs

attempt() { # <log> <cmd...>
    local log="$1" status=0
    shift
    "$@" >"$log" 2>&1 || status=$?
    return "$status"
}

report_of() { # <log> -> the runner's result line, if it reported one
    grep -m1 'test result:' "$1" 2>/dev/null || true
}

busy_note() { # <log>: the runner's own explanation for "the probe was not ours"
    grep -m1 -oE 'could not open interface[^)]*\)|Probe is already in use' "$1" 2>/dev/null || true
}

record() {
    results+=("$1|$2|$3|$4")
}

features_for() { # <mode> -> "a,b", or empty for the plain build
    local mode="$1" feats=()
    if [ "$mode" != default ]; then
        feats+=("$mode")
    fi
    if [ ${#extra_features[@]} -gt 0 ]; then
        feats+=("${extra_features[@]}")
    fi
    join "${feats[@]}"
}

results=()
ok=0
failed=0
runner_failed=0
n=0

run_bin() { # <bin> <mode>; sets log_path and note_out
    local bin="$1" mode="$2" status=0 report feats
    feats=$(features_for "$mode")
    local cmd=(cargo "$test_alias" --test "$bin")
    if [ -n "$feats" ]; then
        cmd+=(--features "$feats")
    fi
    log_path="target/run-logs/${chip}-${width}-${bin}-${mode}.log"
    note_out=""
    attempt "$log_path" "${cmd[@]}" || status=$?
    report=$(report_of "$log_path")

    if [ "$status" -eq 0 ]; then
        record "$bin" "$mode" ok "${report:-no 'test result:' line}"
        ok=$((ok + 1))
        return 0
    fi
    if [ -n "$report" ]; then
        record "$bin" "$mode" FAILED "$report"
        failed=$((failed + 1))
        return 1
    fi

    # Not one result line: the runner never reached the target. The signature of
    # the probe-still-held case is cargo's own "error: test failed" with no test
    # output at all; anything else (a build error, a bad feature name) is not
    # worth retrying.
    if ! grep -q 'error: test failed' "$log_path"; then
        note_out="the run failed before the tests started (see the log)"
        record "$bin" "$mode" FAILED "did not reach the tests"
        failed=$((failed + 1))
        return 1
    fi
    note_out="no 'test result:' line; retrying once in case the probe was still busy"
    sleep 3
    status=0
    attempt "$log_path" "${cmd[@]}" || status=$?
    report=$(report_of "$log_path")

    if [ "$status" -eq 0 ]; then
        note_out="passed on retry (the first attempt produced no test output)"
        record "$bin" "$mode" "ok*" "${report:-no 'test result:' line}"
        ok=$((ok + 1))
        return 0
    fi
    if [ -n "$report" ]; then
        note_out="failed on retry"
        record "$bin" "$mode" FAILED "$report"
        failed=$((failed + 1))
        return 1
    fi
    local busy
    busy=$(busy_note "$log_path")
    if [ -n "$busy" ]; then
        note_out="no test output from the runner after 2 attempts ($busy)"
    else
        note_out="no test output from the runner after 2 attempts"
    fi
    record "$bin" "$mode" RUNNER "no output"
    runner_failed=$((runner_failed + 1))
    return 1
}

run_mode_check() { # <mode>; compile-only, no hardware involved
    local mode="$1" status=0 feats
    feats=$(features_for "$mode")
    local cmd=(cargo "$check_alias")
    # The alias already passes `--tests` (and `--features`), so this only narrows
    # the selection and adds the mode's features.
    if [ ${#bins[@]} -ne ${#BINS[@]} ]; then
        for bin in "${bins[@]}"; do
            cmd+=(--test "$bin")
        done
    fi
    if [ -n "$feats" ]; then
        cmd+=(--features "$feats")
    fi
    log_path="target/run-logs/check-${chip}-${width}-${mode}.log"
    note_out=""
    attempt "$log_path" "${cmd[@]}" || status=$?
    if [ "$status" -eq 0 ]; then
        record "-" "$mode" ok "cargo check --tests"
        ok=$((ok + 1))
        return 0
    fi
    record "-" "$mode" FAILED "cargo check --tests"
    failed=$((failed + 1))
    return 1
}

for mode in "${modes[@]}"; do
    if [ "$compile_only" = true ]; then
        n=$((n + 1))
        printf '[%2d/%d] %-16s ' "$n" "$total" "check $mode"
        if run_mode_check "$mode"; then
            printf 'ok\n'
        else
            printf 'FAILED\n'
            tail -n 20 "$log_path" | sed 's/^/        | /'
        fi
        continue
    fi
    for bin in "${bins[@]}"; do
        n=$((n + 1))
        printf '[%2d/%d] %-12s %-16s ' "$n" "$total" "$bin" "$mode"
        if run_bin "$bin" "$mode"; then
            printf 'ok\n'
        else
            printf 'FAILED\n'
            tail -n 20 "$log_path" | sed 's/^/        | /'
        fi
        if [ -n "$note_out" ]; then
            printf '        note: %s\n' "$note_out"
        fi
    done
done

echo "=========================================="
echo "Summary: $chip, $width_note, $n run(s)"
if [ -n "$selector" ]; then
    # Logged, never stored: the ID belongs to the board that happened to be
    # attached, not to this bench (PLAN.md section 10).
    echo "probe  : $selector"
fi
printf '%-12s %-16s %-7s %s\n' bin mode status report
for r in "${results[@]}"; do
    IFS='|' read -r b m s rep <<<"$r"
    printf '%-12s %-16s %-7s %s\n' "$b" "$m" "$s" "${rep#test result: }"
done
echo "=========================================="
echo "$ok ok, $failed failed, $runner_failed runner failure(s)"
if [ "$failed" -ne 0 ] || [ "$runner_failed" -ne 0 ]; then
    echo "FAILED"
    exit 1
fi
echo "All runs passed!"
