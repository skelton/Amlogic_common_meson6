#!/usr/bin/awk -f
BEGIN{
	words=""
}
/^PAD/ {
	print  words
	words = $0

}

!/^PAD/{
	words = words " " $0
}

END{
	print words
}
