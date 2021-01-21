# Include the nanopb provided Makefile rules
include nanopb.mk

protofile ?= message.proto
out ?= ../

message: message.proto
	@echo "Compiling..."
	@mkdir -p $(out)
	@cp ../lib/nanopb-0.4.4/*.c $(out)
	@cp ../lib/nanopb-0.4.4/*.h $(out)
	@$(PROTOC) $(PROTOC_OPTS) --nanopb_out=$(out) $(protofile)
	@echo "Done."

clean:
	@rm $(out)/*.c
	@rm $(out)/*.h
	@echo "Removed compiled code."