# invoke SourceDir generated makefile for empty_min.pem4f
empty_min.pem4f: .libraries,empty_min.pem4f
.libraries,empty_min.pem4f: package/cfg/empty_min_pem4f.xdl
	$(MAKE) -f C:\Users\dylan\ONEDRI~1\H-Migrated\EGH456_2023\empty_min_EK_TM4C1294XL_TI/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\dylan\ONEDRI~1\H-Migrated\EGH456_2023\empty_min_EK_TM4C1294XL_TI/src/makefile.libs clean

