source gdb_helpers.py

define hookpost-frame
dashboard
end

define hookpost-up_search
dashboard
end

define hookpost-down_search
dashboard
end

define uplib
up_search apvrp::
end

define downlib
down_search apvrp::
end

define run_until_panic
    b rust_panic
    run
    uplib
end

define hook-quit
    set confirm off
end

dashboard -layout breakpoints expressions source stack threads variables
dashboard source -style height 20
dashboard stack -style limit 30
dashboard -style discard_scrollback False

#file target/debug/apvrp
#set env DATA_ROOT = /home/yannik/phd/data
#set args 0 --cpus 1

b rust_panic
b callback.rs:221
run
uplib
