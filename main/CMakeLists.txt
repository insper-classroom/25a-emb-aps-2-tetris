add_executable(pico_emb
        main.c
)

set_target_properties(pico_emb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

target_link_libraries(pico_emb pico_stdlib oled1_lib hardware_adc hardware_i2c Fusion freertos)
pico_add_extra_outputs(pico_emb)
