#!/bin/sh

if command -v python3 &>/dev/null; then
    	echo "Python 3 is installed"
else
    	echo "Python 3 is not installed"
	sudo add-apt-repository ppa:jonathonf/python-3.6
	sudo apt-get update
	sudo apt-get install python3.6
fi

if ! [ -x "$(command -v pip3)" ]; then
	echo "pip is not installed"
	sudo apt install python3-pip
else
	echo "pip is installed"

fi

if ! [ -x "$(command -v pipenv)" ]; then
	echo "pipenv is not installed"
	pip install --user pipenv
else
	echo "pipenv is installed"

fi

~/.local/bin/pipenv install --three
~/.local/bin/pipenv install 'numpy==1.16.2'
~/.local/bin/pipenv install 'scipy==1.2.1'
~/.local/bin/pipenv install 'pyqt5==5.12.1'
~/.local/bin/pipenv install 'matplotlib==3.0.3'
~/.local/bin/pipenv install 'psutil==5.4.2'
~/.local/bin/pipenv install 'opencv-python==3.4.5.20'
~/.local/bin/pipenv lock

currentver="$(blender --version)"
extractedver=$(echo $currentver| cut -d' ' -f 2)
notrequiredver="18.0.0"

if ! [ -x "$(command -v blender)" ]; then
  	echo 'Blender is not installed.' >&2
  	sudo apt-get install blender=2.79.b+dfsg0-1ubuntu1.18.04.1

elif [ "$(printf '%s\n' "$notrequiredver" "$extractedver" | sort -V | head -n1)" = "$notrequiredver" ]; then 
	sudo apt-get install blender=2.79.b+dfsg0-1ubuntu1.18.04.1
else
        echo "Correct blender version"
	echo $currentver
fi

if command -v morse; then
    	echo Morse is installed
else
    	sudo apt-get update
    	sudo apt-get install morse-simulator -y
    	sudo apt-get install python3-morse-simulator -y
fi