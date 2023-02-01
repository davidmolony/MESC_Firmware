# MESC_Firmware
## Foreword:
This project is new as of 28/06/2020, and is the work of David Molony, experienced mechanical and electrical engineer, and software learner.   
Contributions from others welcome, but 
1) The project is not intended to be an all inclusive do everything like VESC. This project is intended to be FOC that "just works" and is trivial to understand, port and build applications with.
2) The project will remain in its entirety BSD 3 clause, MIT or other equivalent entirely permissively licenced.
3) As the project matures, while the code style remains nothing special, there are increasing numbers of originally developed and both effective and easy to implement techniques that are offered permissively. Borrowing sections of code for other projects is allowed, but without exception, if you borrow the code, even if you rename variables or break it up/relocate into various subroutines, you MUST credit the origin, and maintain the permissive BSD licencing inline if necessary. Failure to do so means you grant a perpetual permissive licence to your project and lose your rights to using this code permissively. This statement is in response to a pernicious yet ubiquitous habit of open source projects ripping out permissive licencing or taking public domain code and re-issuing under copyleft terms. The origin of this ire was searching for python source and finding the entire cpython project online (since removed) with all trace of the BSD/PSF licence removed and replaced with GPL.

**Thanks to contributors, especially:**
c0d3b453 for large amounts of helpwith C and teaching, motor, speed temp and other profiles
Salavat for initial STM32 setup and teaching,
Elwin (offline) for testing, motor control idea bouncing and assistance with current controllers,
Jens (Netzpfuscher) for RTOS, Terminal and variable save v2, tidying up and contributions to SinLUT,
OWhite for creating the Github Pages