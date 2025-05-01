# LEAPS UDK-Example

This repository contains a LEAPS UDK's example application.

This repository is versioned together with the [Zephyr main tree][zephyr]. 
This means that every time that Zephyr is tagged, this repository is tagged as 
well with the same version number, and the [manifest](west.yml) entry for 
`zephyr` will point to the corresponding Zephyr tag. For example, 
`example-application-application` v2.6.0 will point to Zephyr v2.6.0. 

Note that the `main` branch will always point to the development branch of 
Zephyr, also `main`.

[board_porting]: https://docs.zephyrproject.org/latest/guides/porting/board_porting.html
[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr

# Getting Started

To assist users in easily setting up and running the examples, LEAPS has 
provided a detailed guideline in the following document: 
[UDK-SDK Getting Started Guide](https://docs.leapslabs.com/udk/development-guide/udk-sdk-getting-started/). 
Please refer to this guide for each respective platform.


# Qorvo's Licensing

Please visit file license.txt before referring to Qorvo's file
