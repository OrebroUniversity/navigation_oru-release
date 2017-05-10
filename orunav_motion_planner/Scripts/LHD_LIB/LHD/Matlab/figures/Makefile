fn = fig_pdf

all: 
	latex $(fn).tex
	dvips -o $(fn).ps $(fn).dvi
	ps2pdf -sPAPERSIZE=a4 $(fn).ps $(fn).pdf

clean:
	rm -f $(fn).dvi
	rm -f $(fn).ps
	rm -f $(fn).out	
	rm -f $(fn).log	
	rm -f $(fn).aux	
	rm -f a

# dummy targets
.PHONY: clean
