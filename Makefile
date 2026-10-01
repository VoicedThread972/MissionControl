PROJECT_NAME := MissionControl
MC_MITM_TID := 010000000000bd00

GIT_HASH := $(shell git rev-parse --short HEAD)$(shell git diff --quiet HEAD -- . || printf '%s' '-dirty')
# Explicit backport baseline: fetching a tag must not change the build version.
VERSION := 0x000F02
BUILD_VERSION := 0.15.2-sw2-experimental-$(GIT_HASH)
BUILD_DATE := $(shell date)

TARGETS := mcmitm_version.cpp mc_mitm

all: $(TARGETS)

mcmitm_version.cpp: .git/HEAD .git/index
	echo "namespace ams::mc { unsigned int mc_version = $(VERSION); const char *mc_build_name = \"$(BUILD_VERSION)\"; const char *mc_build_date = \"$(BUILD_DATE)\"; }" > mc_mitm/source/$@

mc_mitm: mcmitm_version.cpp
	$(MAKE) -C $@

test:
	$(MAKE) -C tests test

clean:
	$(MAKE) -C mc_mitm clean
	rm mc_mitm/source/mcmitm_version.cpp
	rm -rf dist

dist: all
	rm -rf dist

	

	mkdir -p dist/atmosphere/contents/$(MC_MITM_TID)
	cp mc_mitm/out/nintendo_nx_arm64_armv8a/release/mc_mitm.nsp dist/atmosphere/contents/$(MC_MITM_TID)/exefs.nsp
	echo "btdrv" >> dist/atmosphere/contents/$(MC_MITM_TID)/mitm.lst
	echo "btm" >> dist/atmosphere/contents/$(MC_MITM_TID)/mitm.lst

	mkdir -p dist/atmosphere/contents/$(MC_MITM_TID)/flags
	touch dist/atmosphere/contents/$(MC_MITM_TID)/flags/boot2.flag

	cp mc_mitm/toolbox.json dist/atmosphere/contents/$(MC_MITM_TID)/toolbox.json

	cp -r exefs_patches dist/atmosphere/

	mkdir -p dist/config/MissionControl
	mkdir -p dist/config/MissionControl/controllers
	cp mc_mitm/config.ini dist/config/MissionControl/missioncontrol.ini.template
	cp SWITCH2_STATUS.md dist/SWITCH2_STATUS.md
	cp LICENSE dist/LICENSE
	cd dist; find . -type f ! -name SHA256SUMS -exec sha256sum {} \; > SHA256SUMS

	cd dist; zip -r $(PROJECT_NAME)-$(BUILD_VERSION).zip ./*; cd ../;

.PHONY: all clean dist test $(TARGETS)
