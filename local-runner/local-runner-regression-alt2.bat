start java -Xms512m -Xmx1G -jar "local-runner.jar" local-runner-regression-alt2.properties local-runner-regression-alt2.default.properties
@echo "wait for java start..."
@timeout 10

start /min cpp-cgdk.exe 127.0.0.1 31031 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31032 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31033 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31034 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31035 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31036 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31037 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31038 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31039 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31040 0000000000000000
@timeout 5

@echo "Done"
pause

