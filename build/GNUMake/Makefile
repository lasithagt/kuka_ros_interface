BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

TARGETLIB 		= libFRIClient.a

SOURCE_DIRS		= $(CLIENTBASE_DIR) \
				  	  $(CLIENTLBR_DIR) \
				 	  $(CONNECTION_DIR) \
					  $(NANOPB_DIR) \
				 	  $(PROTOBUF_DIR) \
				 	  $(PROTOBUF_GEN_DIR)

EXAMPLE_DIRS 	= $(EXP_DIR)/LBRJointSineOverlay\
 					  $(EXP_DIR)/LBRTorqueSineOverlay\
					  $(EXP_DIR)/LBRWrenchSineOverlay\
					  $(EXP_DIR)/LBRClientTemplate

SOURCE_DIRS_OBJS	= $(SOURCE_DIRS:%=%/$(OBJ_DIR)/*.o)



all: lib examples

clean: clean_lib

lib:
	@if [ -n "$(SOURCE_DIRS)" ] ; then  \
  	echo " "; \
	for i in $(SOURCE_DIRS); do  \
		$(MAKE) -C $$i all; \
	done; \
	fi;
	$(MKDIR) $(LIB_DIR)
	$(AR) cr $(LIB_DIR)/$(TARGETLIB) $(SOURCE_DIRS_OBJS)

clean_lib:
	@if [ -n "$(SOURCE_DIRS)" ] ; then  \
  	echo " "; \
	for i in $(SOURCE_DIRS); do  \
		$(MAKE) -C $$i clean; \
	done; \
	fi;
	$(RM) $(LIB_DIR)/$(TARGETLIB)


doxygen:
	cd $(DOC_DIR); doxygen Doxyfile;

.PHONY: all lib clean clean_lib doxygen
