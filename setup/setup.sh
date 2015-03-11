#!/usr/bin/env bash
echo -e "# ============================================================
# Added by Eric
alias getin='sudo apt-get install'
alias getreps='apt-cache search'
alias getrem='sudo apt-get autoremove'
alias getlocs='dpkg -l'
alias getupd='sudo apt-get update'
alias getupg='sudo apt-get upgrade'
alias getdupg='sudo apt-get dist-upgrade'
alias sl='sl -e'
alias rm='rm -i'
alias cp='cp -iv'
alias mv='mv -iv'
alias gits='git status -s'

export PS1='\[\e[1;96m\]\W\[\e[m\] \[\e[1;92m\]\$\[\e[m\] \[\e[0m\]'  

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
catmake() {
    current_dir=$(pwd)
    roscd
    cd ..
catkin_make
    cd $current_dir
}

cdls() { cd "$@" && ls; }
cpplint() {
    current_dir=$(pwd)
    cd ~/Downloads
    ./cpplint.py "$current_dir/$1"
    cd $current_dir
}

run_tests() {
    test_files=( $(ls | grep 'test_.*\.py$') )

    for test_file in "${test_files[@]}"
    do
        echo "Printing results from $test_file"
        python $test_file
    done

    rm -f *.pyc
}
# ============================================================" > ~/.bash_aliases


# To enable all ubuntu software (main universe restricted multiverse)
# http://askubuntu.com/questions/148638/how-do-i-enable-the-universe-repository
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe restricted multiverse"


# To begin the chrome install
# http://ubuntuportal.com/2014/04/how-to-install-google-chrome-web-browser-in-ubuntu-14-04-lts-trusty-tahr.html
cd ~/Downloads
if [ ! -f ~/Downloads/google-chrome-stable_current_amd64.deb ]; then
	wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
fi
sudo dpkg -i google-chrome-stable_current_amd64.deb
sudo apt-get update
sudo apt-get upgrade
sudo apt-get -f install
sudo dpkg -i google-chrome-stable_current_amd64.deb


# To get sublime
sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install -y sublime-text-installer


# To get cpplint
# http://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py
cd ~/Downloads
if [ ! -f ~/Downloads/cpplint.py ]; then
	wget http://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py
fi


# Set up APM Mission Planner
cd ~/Downloads
wget http://ardupilot.com/wp-content/plugins/download-monitor/download.php?id=111 -O apmplanner2.deb
sudo dpkg -i apmplanner2.deb
sudo apt-get -f install
sudo dpkg -i apmplanner2.deb
rm -f apmplanner2.deb

# Various useful installs
sudo apt-get install -y clang
sudo apt-get install -y xclip
sudo apt-get install -y vim
sudo apt-get install -y sl
sudo apt-get install -y screen
sudo apt-get install -y gimp
sudo apt-get install -y inkscape
sudo apt-get install -y gparted
sudo apt-get install -y pep8
sudo apt-get install -y build-essential
sudo apt-get install -y automake
sudo apt-get install -y git
sudo apt-get install -y git-core
sudo apt-get install -y python-prettytable
sudo apt-get install -y libsdl1.2debian
sudo apt-get install -y libqt5serialport5
sudo apt-get install -y ipython
sudo apt-get install -y python-pip
sudo apt-get install -y ros-indigo-geodesy
sudo apt-get update
sudo apt-get upgrade

sudo pip install mock

# Set up github stuff
# SSH key
ssh-keygen -t rsa -C "arl.olin.scope@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
xclip -sel clip < ~/.ssh/id_rsa.pub

echo "\n SET UP YOUR GIT KEYS ON GITHUB"
echo "https://help.github.com/articles/generating-ssh-keys"

echo "RUN THE LINE sudo adduser username dialout FOR YOUR USERNAME\n\n"


