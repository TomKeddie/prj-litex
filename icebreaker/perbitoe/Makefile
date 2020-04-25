all: origin branch perbitoe diff

origin: singlebitoe.py
	python3 singlebitoe.py /home/tom/git/m-labs/migen origin

branch: singlebitoe.py
	python3 singlebitoe.py /home/tom/git/TomKeddie/migen branch

perbitoe: perbitoe.py
	python3 perbitoe.py /home/tom/git/TomKeddie/migen perbitoe

clean:
	rm -rf origin branch perbitoe diff-*

diff: diff_origin_branch diff_origin_perbitoe

diff_origin_branch:
	diff origin/arty_a7_build/top.v     branch/arty_a7_build/
	diff origin/icebreaker_build/top.v  branch/icebreaker_build/
	diff origin/papilio_pro_build/top.v branch/papilio_pro_build/
	diff origin/versaecp55g_build/top.v branch/versaecp55g_build/

diff_origin_perbitoe:
	-diff origin/arty_a7_build/top.v     perbitoe/arty_a7_build/
	-diff origin/icebreaker_build/top.v  perbitoe/icebreaker_build/
	-diff origin/papilio_pro_build/top.v perbitoe/papilio_pro_build/
	-diff origin/versaecp55g_build/top.v perbitoe/versaecp55g_build/
