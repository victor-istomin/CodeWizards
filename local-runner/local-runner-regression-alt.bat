start java -Xms512m -Xmx1G -jar "local-runner.jar" local-runner-regression-alt.properties local-runner-regression-alt.default.properties
@echo "wait for java start..."
@timeout 10

start /min cpp-cgdk.exe 127.0.0.1 31021 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31022 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31023 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31024 0000000000000000
@timeout 5

start /min cpp-cgdk.exe 127.0.0.1 31025 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31026 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31027 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31028 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31029 0000000000000000
@timeout 5

start /min cpp-cgdk-old.exe 127.0.0.1 31030 0000000000000000
@timeout 5

@echo "Done"
pause

