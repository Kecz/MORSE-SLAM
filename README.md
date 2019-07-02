### To install packages required for proper usage of project use:
    cd morse
    chmod +x install.sh
    ./install.sh

### To run simulation in blender:
    cd morse
    cd MORSEProject
    morse run env.py

#### To run GUI with progress of mapping:
    cd morse
    cd MORSEProject
    pipenv run python GUI_.py
   
In GUI you have to firstly run blender using 'Prepare Environment' button.
Next you can choose how long in seconds the mapping will took by entering right value in window next to 'Prepare Environment' button.  Mapping will start after pressing 'Run simulation'. 

After that you will see live updating plots of mapped terrain in 2D and 3D and based on Pose and  on estimated trajectory.

    
#### To change enviroment
Edit file env.py and change path to selected enviroment, like that:
    
    env = Environment('env_3D/Rampa_duza_bez_sufitu2.blend', fastmode=False)
    
Other advanced environments can be found in 'env_3D' file.
Simple environments can be found in 'env_simple'.
### Git global setup
    git config --global user.name "Imie Nazwisko"
    git config --global user.email "adres@email.com"

##### Create a new repository
    git clone https://gitlab.com/wojrut97/morse.git
    cd morse
    touch README.md
    git add README.md
    git commit -m "add README"

##### Existing folder
    cd existing_folder
    git init
    git remote add origin https://gitlab.com/wojrut97/morse.git
    git add .
    git commit -m "Initial commit"

##### Existing Git repository
    cd existing_repo
    git remote rename origin old-origin
    git remote add origin https://gitlab.com/wojrut97/morse.git
  
##### To switch branch:
    git checkout name-of-branch

