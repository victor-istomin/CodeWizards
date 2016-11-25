start java -Xms512m -Xmx1G -jar "local-runner.jar" local-runner-regression.properties local-runner-regression.default.properties
@echo "wait for java start..."
@timeout 10

start /min cpp-cgdk.exe 127.0.0.1 31001 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31002 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31003 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31004 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31005 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31006 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31007 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31008 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31009 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31010 0000000000000000
@timeout 5

@echo "Done"
pause

