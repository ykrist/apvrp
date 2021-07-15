import gdb
import re

STDLIB = [
    r"<?std::",
    r"<?core::",
    r"rust_begin_unwind",
]
STDLIB = [re.compile(r) for r in STDLIB]

def navigate_stack(traverse_fn, whitelist, blacklist):
    frame = gdb.selected_frame()
    while True:
        frame = traverse_fn(frame)
        if frame is None:
            break

        if any(r.match(frame.name()) for r in whitelist):
            break

        if len(blacklist) > 0 and not any(r.match(frame.name()) for r in blacklist):
            break

    if frame is None:
        print("no more frames found")
    else:
        frame.select()

def navigate_up(whitelist, blacklist):
    navigate_stack(lambda f: f.older(), whitelist, blacklist)

def navigate_down(whitelist, blacklist):
    navigate_stack(lambda f: f.newer(), whitelist, blacklist)

class NavigateUpStack(gdb.Command):
    def __init__(self):
        super(NavigateUpStack, self).__init__("up_skip_stdlib", gdb.COMMAND_STACK)

    def invoke(self, _args, _from_tty):
        navigate_up([], STDLIB)

class NavigateDownStack(gdb.Command):
    def __init__(self):
        super(NavigateDownStack, self).__init__("down_skip_stdlib", gdb.COMMAND_STACK)

    def invoke(self, _args, _from_tty):
        navigate_down([], STDLIB)

class SearchUpStack(gdb.Command):
    def __init__(self):
        super(SearchUpStack, self).__init__("up_search", gdb.COMMAND_STACK)

    def invoke(self, regexp, _from_tty):
        navigate_up([re.compile(regexp)], [])

class SearchDownStack(gdb.Command):
    def __init__(self):
        super(SearchDownStack, self).__init__("down_search", gdb.COMMAND_STACK)

    def invoke(self, regexp, _from_tty):
        navigate_down([re.compile(regexp)], [])

NavigateUpStack()
NavigateDownStack()
SearchUpStack()
SearchDownStack()
