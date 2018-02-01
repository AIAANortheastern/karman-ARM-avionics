# karman-ARM-avionics
ARM Development


### Code Composer Studio

The project requires TI's Code Composer Stuido Version 7 (ccsv7). This is necessary for
the cross-compiler as well as TI's SimpleLink SDK.

* Download Code composer studio version 7.x [here](http://processors.wiki.ti.com/index.php/Download_CCS)
* Ensure that Code Composer studio will be installed at ~/ti (or $HOME/ti or /home/<username>/ti) on linux, or C:/ti on Windows.
* Click the checkbox for SimpleLink MSP432
* Perform all  other installation steps per TI's wiki as linked above
* On the "Getting Started" page, click the "Browse Examples" option to open the Resource Explorer. Alternatively, if the Resource Explorer is already open, navigate to that tab.
* Open the tree on the left side of the Resource Explorer titled "Software"
* Click the button that looks like a download button and download the MSP432 SDK v:1.60.00.12 "offline"

### FreeRTOS

FreeRTOS is used to manage task scheduling

* Download FreeRTOSv10.0.0 [here](http://www.freertos.org/a00104.html)
* Extract the archive to either your home directory on linux, or c:/ on Windows

### Adding the projects to Code Composer Studio

Steps to start working:
 
 * Clone the git repository to your favorite location

 * Download ccsv7 with MSP432 and CC13xx libraries
 	* /home/user/ti
 	* c:\ti
 
 * Install MSP432 simplelink sdk v1.60.00.12 from resource explorer
 
 * Install FreeRTOSv10.0.0
 	* /home/user/FreeRTOSv10.0.0
 	* c:\FreeRTOSv10.0.0
 
 * Open CCS and create a workspace in the same directory as the git repo
 
 * Go to Window-->Preferences-->General-->Workspace-->Linked Resources
     * New, Name = FREERTOS_INSTALL_DIR, Location = FreeRTOS Install Folder (with the vXX.XX.XX)
	 
 * Go to Project-->Import CCS Projects
     * Click Browse, Ok (Defaults to workspace directory)
	 * Check box next to all projects, click finish
	 	 
 * Build the karman-application project, or your favorite application project
 
 * Right click the project in the project explorer
	 * Debug as --> code composer debug session
	 * Stop the debug session
	 * Go back to Debug as ---> Debug Configurations --> Target
	 * Uncheck Enable Semihosting
 
 * Happy coding! :)
 