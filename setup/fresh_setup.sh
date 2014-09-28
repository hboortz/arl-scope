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
PS1=’$:’

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


# Various useful installs
sudo apt-get install -y clang
sudo apt-get install -y xclip
sudo apt-get install -y vim
sudo apt-get install -y sl
sudo apt-get install -y gimp
sudo apt-get install -y inkscape
sudo apt-get install -y gparted
sudo apt-get install -y pep8
sudo apt-get install -y build-essential
sudo apt-get install -y automake
sudo apt-get install -y git
sudo apt-get install -y git-core
sudo apt-get update
sudo apt-get upgrade


# Set up github stuff
# SSH key
ssh-keygen -t rsa -C "arl.olin.scope@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
xclip -sel clip < ~/.ssh/id_rsa.pub

echo "\n SET UP YOUR GIT KEYS ON GITHUB"
echo "https://help.github.com/articles/generating-ssh-keys"

echo "RUN THE LINE sudo adduser username dialout FOR YOUR USERNAME\n\n"
