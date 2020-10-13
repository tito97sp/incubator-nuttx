include $(TOPDIR)/Make.defs

include Make.defs

CXXFLAGS += ${shell $(INCDIR) "$(CXX)" $(TOPDIR)$(DELIM)libs$(DELIM)uorb_msgs}
CXXFLAGS += -std=c++11

BINDIR ?= bin

CPPSRCS =  $(uorb_sources)

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

CXXOBJS = $(patsubst %.cxx, $(BINDIR)$(DELIM)%$(OBJEXT), $(CXXSRCS))
CPPOBJS = $(patsubst %.cpp, $(BINDIR)$(DELIM)%$(OBJEXT), $(CPPSRCS))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS) $(CPPSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS) $(CPPOBJS)

BIN = libuorb_msgs$(LIBEXT)


all: $(BIN)

.PHONY: depend clean distclean .autogen

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(CPPOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))


.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >bin/Make.dep
	$(Q) touch $@
	
.autogen:
	@python tools/px_generate_uorb_topic_files.py  \
	--headers \
	-f $(list_msg_files) \
	-i $(uorb_msgs_path) \
 	-e $(uorb_msgs_path)/templates/uorb_posix \
 	-o ${msg_headers_out_path} \
 	-t ${msg_tmp_dir}/headers \
	-q

	@python tools/px_generate_uorb_topic_files.py  \
	--sources \
	-f $(list_msg_files) \
	-i $(uorb_msgs_path) \
 	-e $(uorb_msgs_path)/templates/uorb_posix \
 	-o ${msg_sources_out_path} \
 	-t ${msg_tmp_dir}/sources \
	-q 
	
depend: .autogen .depend

clean:
	$(Q) $(MAKE) -C bin  clean
	#$(Q) $(MAKE) -C zoneinfo clean BIN=$(BIN)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(uorb_sources_path))
	$(call DELFILE, $(uorb_headers_path))
	$(call DELDIR, $(msg_msg_dir))
	$(call CLEAN)

distclean: clean
	$(Q) $(MAKE) -C bin  distclean
	#$(Q) $(MAKE) -C zoneinfo distclean BIN=$(BIN)
	$(call DELFILE, bin/Make.dep)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include bin/Make.dep
-include Make.depa
