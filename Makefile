.PHONY: help test ci schematics clean

help:
	@echo "make test        run your testbenches (writes results/local.txt)"
	@echo "make ci          same thing, under the name CI uses (results/ci.txt)"
	@echo "make schematics  synthesize alu/imm/rf/decoder/cache/hart with yosys and"
	@echo "                 render gate-level SVGs with netlistsvg (build/schematics/)"
	@echo "make clean       remove results/ and build/"
	@echo
	@echo "Details:      ./run_test.sh <name> [submission_dir]"
	@echo "Write your own <check>_tb.v for alu, imm, rf_bypass, rf_no_bypass,"
	@echo "decoder, cache, hart_base, hart_direct_mapped, hart_set_assoc,"
	@echo "hart_plru and hart_prefetch -- see README.md and example/opmux_tb.v."
	@echo "Run pip install ./lint once so make test also runs the ece552 rule"
	@echo "checker. There is no autograder for phase 5; this only runs the"
	@echo "testbenches you write."

test:
	@./run_test.sh local

ci:
	@./run_test.sh ci

schematics:
	@./gen_schematics.sh

clean:
	rm -rf results build
