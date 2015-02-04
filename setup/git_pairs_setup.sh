sudo apt-get install -y ruby
cd /usr/local/bin && curl -L http://github.com/pivotal/git_scripts/tarball/master | gunzip | sudo tar xvf - --strip=2
cd
echo "pairs:
    km: Kyle McConnaughay; kyle.mcconnaughay
    cg: Charles Goddard; chargoddard
    es: Eric Schneider; franz.eric.schneider
    ed: Elizabeth Duncan; truepinkluv24
    kg: Kaitlin Gallagher; kkgallagher9
email:
    domain: gmail.com
global:
    true" > .pairs
