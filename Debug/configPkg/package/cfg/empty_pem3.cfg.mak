# invoke SourceDir generated makefile for empty.pem3
empty.pem3: .libraries,empty.pem3
.libraries,empty.pem3: package/cfg/empty_pem3.xdl
	$(MAKE) -f C:\Users\Tuomas\Documents\Opiskelu_repot\johdatus_tietokonejarjestelmiin\Harjoitustyo/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Tuomas\Documents\Opiskelu_repot\johdatus_tietokonejarjestelmiin\Harjoitustyo/src/makefile.libs clean

