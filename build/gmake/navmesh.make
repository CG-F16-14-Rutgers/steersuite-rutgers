# GNU Make project makefile autogenerated by Premake
ifndef config
  config=debug
endif

ifndef verbose
  SILENT = @
endif

CC = clang
CXX = clang++
AR = ar

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),debug)
  OBJDIR     = obj/Debug/navmesh
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libnavmesh.dylib
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DDEBUG
  INCLUDES  += -I../../steerlib/include -I../../navmeshBuilder/include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../steersimlib/include -I../../external -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -fPIC -stdlib=libc++ -std=c++0x -ggdb -fPIC
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -dynamiclib -stdlib=libc++ -Wl,-rpath,/Users/russ/Desktop/A6/steersuite-rutgers/build/lib -install_name @rpath/libnavmesh.dylib
  LDDEPS    += ../lib/libsteerlib.dylib ../lib/libsteersimlib.dylib ../lib/libutil.dylib ../lib/libRecast.a ../lib/libDebugUtils.a ../lib/libDetour.a ../lib/libDetourCrowd.a
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release)
  OBJDIR     = obj/Release/navmesh
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libnavmesh.dylib
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DNDEBUG
  INCLUDES  += -I../../steerlib/include -I../../navmeshBuilder/include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../steersimlib/include -I../../external -I../../util/include
  ALL_CPPFLAGS  += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS    += $(CFLAGS) $(ALL_CPPFLAGS) $(ARCH) -Wall -Wextra -g -O2 -fPIC -stdlib=libc++ -std=c++0x -ggdb -fPIC
  ALL_CXXFLAGS  += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS  += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  ALL_LDFLAGS   += $(LDFLAGS) -L. -L../lib -dynamiclib -stdlib=libc++ -Wl,-rpath,/Users/russ/Desktop/A6/steersuite-rutgers/build/lib -install_name @rpath/libnavmesh.dylib
  LDDEPS    += ../lib/libsteerlib.dylib ../lib/libsteersimlib.dylib ../lib/libutil.dylib ../lib/libRecast.a ../lib/libDebugUtils.a ../lib/libDetour.a ../lib/libDetourCrowd.a
  LIBS      += $(LDDEPS) -framework OpenGL
  LINKCMD    = $(CXX) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/ChunkyTriMesh.o \
	$(OBJDIR)/ConvexVolumeTool.o \
	$(OBJDIR)/CrowdTool.o \
	$(OBJDIR)/imgui.o \
	$(OBJDIR)/imguiRenderGL.o \
	$(OBJDIR)/InputGeom.o \
	$(OBJDIR)/Main.o \
	$(OBJDIR)/Mesh.o \
	$(OBJDIR)/MeshLoaderObj.o \
	$(OBJDIR)/NavMeshModule.o \
	$(OBJDIR)/NavMeshPruneTool.o \
	$(OBJDIR)/NavMeshTesterTool.o \
	$(OBJDIR)/OffMeshConnectionTool.o \
	$(OBJDIR)/PerfTimer.o \
	$(OBJDIR)/RecaseNavMeshPlanner.o \
	$(OBJDIR)/Sample.o \
	$(OBJDIR)/Sample_SoloMesh.o \
	$(OBJDIR)/SampleInterfaces.o \
	$(OBJDIR)/Timer.o \
	$(OBJDIR)/ValueHistory.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking navmesh
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning navmesh
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
	$(SILENT) $(CXX) -x c++-header $(ALL_CXXFLAGS) -MMD -MP $(DEFINES) $(INCLUDES) -o "$@" -MF "$(@:%.gch=%.d)" -c "$<"
endif

$(OBJDIR)/ChunkyTriMesh.o: ../../navmeshBuilder/src/ChunkyTriMesh.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/ConvexVolumeTool.o: ../../navmeshBuilder/src/ConvexVolumeTool.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/CrowdTool.o: ../../navmeshBuilder/src/CrowdTool.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/imgui.o: ../../navmeshBuilder/src/imgui.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/imguiRenderGL.o: ../../navmeshBuilder/src/imguiRenderGL.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/InputGeom.o: ../../navmeshBuilder/src/InputGeom.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Main.o: ../../navmeshBuilder/src/Main.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Mesh.o: ../../navmeshBuilder/src/Mesh.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/MeshLoaderObj.o: ../../navmeshBuilder/src/MeshLoaderObj.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/NavMeshModule.o: ../../navmeshBuilder/src/NavMeshModule.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/NavMeshPruneTool.o: ../../navmeshBuilder/src/NavMeshPruneTool.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/NavMeshTesterTool.o: ../../navmeshBuilder/src/NavMeshTesterTool.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/OffMeshConnectionTool.o: ../../navmeshBuilder/src/OffMeshConnectionTool.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/PerfTimer.o: ../../navmeshBuilder/src/PerfTimer.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/RecaseNavMeshPlanner.o: ../../navmeshBuilder/src/RecaseNavMeshPlanner.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Sample.o: ../../navmeshBuilder/src/Sample.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Sample_SoloMesh.o: ../../navmeshBuilder/src/Sample_SoloMesh.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/SampleInterfaces.o: ../../navmeshBuilder/src/SampleInterfaces.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/Timer.o: ../../navmeshBuilder/src/Timer.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

$(OBJDIR)/ValueHistory.o: ../../navmeshBuilder/src/ValueHistory.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CXXFLAGS) $(FORCE_INCLUDE) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)
ifneq (,$(PCH))
  -include $(OBJDIR)/$(notdir $(PCH)).d
endif
