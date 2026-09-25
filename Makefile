BLENDER := /Applications/Blender.app/Contents/MacOS/Blender
BLEND   := build/nyx.blend
SAMPLES ?= 64

.PHONY: all build verify aero render export web stl manifest vendor clean

all: build verify render web

vendor:                      ## re-copy the Aether AX-1 from ../aether-ax1
	python3 tools/vendor_aether.py ../aether-ax1

build:                       ## generate geometry, assemble build/nyx.blend
	$(BLENDER) --background --factory-startup --python nyx/assemble.py
	python3 tools/make_manifest.py

aero:                        ## area rule, vortex lattice, turn performance
	python3 aero/area_rule.py
	python3 aero/vlm.py
	python3 aero/agility.py

verify:                      ## the definition of done
	python3 nyx/verify.py
	python3 tools/audit_watertight.py
	python3 tools/audit_geometry.py
	python3 tools/audit_structure.py
	python3 tools/audit_intersect.py
	python3 tools/audit_stowage.py
	python3 tools/measure_fuel.py --check
	python3 tools/audit_oml.py
	python3 tools/audit_support.py
	python3 tools/audit_joints.py
	python3 tools/audit_manifest.py
	node tools/validate_viewer.mjs .

render:                      ## hero, plan, side
	$(BLENDER) -b $(BLEND) -P nyx/render.py -- all $(SAMPLES)

export:
	$(BLENDER) -b $(BLEND) -P nyx/export.py -- glb

web:                         ## decimated, Draco-compressed GLB for the viewer
	$(BLENDER) -b $(BLEND) -P nyx/export.py -- web

stl:
	$(BLENDER) -b $(BLEND) -P nyx/export.py -- stl

manifest:
	python3 tools/make_manifest.py

clean:
	rm -rf build/*.blend build/*.glb
