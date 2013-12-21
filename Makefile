# Makefile
all: latex

latex:
	latex companion_document.tex
	latex companion_document.tex
	latex companion_document.tex
	dvips companion_document.dvi
	pstopdf companion_document.ps
	rm -f *.log *.aux *.toc *.dvi *.ps
	open -a Preview companion_document.pdf
