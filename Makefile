req:
	mkdir -p results
	pip install -r requirements.txt

gitrcommit:
	rm -rf .git/hooks/pre-push .git/hooks/post-push
	rm -rf .git/hooks/post-commit
	rm -rf .git/hooks/post-merge .git/hooks/post-checkout
	git config --global http.sslVerify false
	git config --global credential.helper store
	# git add -u
	git add Makefile README.md
	-git add pingpongsim/*.py results/*.py
	-git commit -a -m "`date`"
	git pull --no-rebase
	git push origin HEAD

gitrupdate:
	rm -rf .git/hooks/pre-push .git/hooks/post-push
	rm -rf .git/hooks/post-commit
	rm -rf .git/hooks/post-merge .git/hooks/post-checkout
	git config --global http.sslVerify false
	git config --global credential.helper store
	git pull --no-rebase

