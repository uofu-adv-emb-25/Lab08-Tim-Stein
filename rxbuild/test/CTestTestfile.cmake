# CMake generated Testfile for 
# Source directory: /Users/stein/adv-emb/lab8/test
# Build directory: /Users/stein/adv-emb/lab8/rxbuild/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(simulate_mytest "RENODE-NOTFOUND" "--disable-xwt" "--port" "-2" "--pid-file" "renode.pid" "--console" "-e" "\$ELF=@/Users/stein/adv-emb/lab8/rxbuild/test/mytest.elf; \$WORKING=@/Users/stein/adv-emb/lab8; include @/Users/stein/adv-emb/lab8/test/simulate.resc; start")
set_tests_properties(simulate_mytest PROPERTIES  _BACKTRACE_TRIPLES "/Users/stein/adv-emb/lab8/test/CMakeLists.txt;39;add_test;/Users/stein/adv-emb/lab8/test/CMakeLists.txt;0;")
