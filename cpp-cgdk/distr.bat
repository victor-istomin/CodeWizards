@set distr=cpp-cgdk.zip

del /f /q %distr%
"%programfiles%"\7-Zip\7z a %distr% *.h *.hpp *.cpp -x!Remote*.* -x!Strategy.* -x!Runner.*

@echo "Done: %distr%"
@pause
