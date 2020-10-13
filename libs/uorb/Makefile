############################################################################
# drivers/Makefile
#
#   Copyright (C) 2007-2014, 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include $(TOPDIR)/Make.defs


CPPSRCS += uORB.cpp Subscription.cpp uORBDeviceMaster.cpp uORBDeviceNode.cpp 
CPPSRCS += uORBManager.cpp uORBUtils.cpp uORBTopics.cpp

CXXFLAGS += ${shell $(INCDIR) "$(CXX)" $(TOPDIR)$(DELIM)libs$(DELIM)uorb}

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))
CPPOBJS = $(CPPSRCS:.cpp=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS) $(CPPSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS) $(CPPOBJS)

BIN = libuorb$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)


$(CXXOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(CPPOBJS): %$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend
	
clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
