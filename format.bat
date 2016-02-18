@rem skips source control metadata folders and hidden files (.*)

@rem format C++ code
dir /s /b src | findstr /v "^\." | findstr "\.cpp$ \.hpp$ \.inl$" | uncrustify.exe -c uncrustify.cfg -F - -l CPP --replace --no-backup

@rem format C code
dir /s /b src | findstr /v "^\." | findstr "\.c$ \.h$" | uncrustify.exe -c uncrustify.cfg -F - -l C --replace --no-backup
