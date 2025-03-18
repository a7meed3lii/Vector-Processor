vlib work
vlog *.*v
vsim -c -voptargs=+acc work.tb_vector_processor
add wave *
run -all