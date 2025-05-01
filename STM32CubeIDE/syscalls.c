/**
 * @file _syscalls.c
 * @brief Stub implementations for system calls required by newlib.
 *
 * This file provides minimal implementations of system calls used by the C standard library (newlib).
 * These functions are required for functions like printf(), scanf(), and file operations.
 *
 * Since STM32 microcontrollers do not have an operating system or a traditional filesystem,
 * these system calls are implemented as stubs to prevent linker errors and warnings.
 *
 * If needed, some functions (e.g., _write) can be modified to work with peripherals like UART.
 */

#include <sys/stat.h>
#include <unistd.h>

/**
 * @brief Write function (used by printf).
 * 
 * This function is called by newlib's printf() and other output functions.
 * Modify it to use UART if you want printf() output.
 *
 * @param file File descriptor (ignored)
 * @param ptr Pointer to data to write
 * @param len Number of bytes to write
 * @return Number of bytes written, or -1 on error
 */
__attribute__((weak)) int _write(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return -1;
}

/**
 * @brief Read function (used by scanf).
 * 
 * This function is called when reading input (e.g., from stdin).
 * Modify it if using an input peripheral.
 *
 * @param file File descriptor (ignored)
 * @param ptr Pointer to buffer to store input
 * @param len Number of bytes to read
 * @return Number of bytes read, or -1 on error
 */
__attribute__((weak)) int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return -1;
}

/**
 * @brief Change file position (stub for lseek).
 * 
 * This function is used for seeking in files. Not needed in embedded systems.
 *
 * @param file File descriptor (ignored)
 * @param ptr Offset (ignored)
 * @param dir Direction (ignored)
 * @return Always returns -1
 */
__attribute__((weak)) int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return -1;
}

/**
 * @brief Process termination signal (stub for kill).
 * 
 * This function is required for process management, which is not used in embedded systems.
 *
 * @param pid Process ID (ignored)
 * @param sig Signal (ignored)
 * @return Always returns -1
 */
__attribute__((weak)) int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    return -1;
}

/**
 * @brief Check if a file descriptor is a terminal (stub for isatty).
 * 
 * This function is used to determine if a file descriptor refers to a terminal.
 * Always returns 0 (not a terminal).
 *
 * @param file File descriptor (ignored)
 * @return Always returns 0
 */
__attribute__((weak)) int _isatty(int file) {
    (void)file;
    return 0;
}

/**
 * @brief Get process ID (stub for getpid).
 * 
 * Since there is no process management in embedded systems, always returns 1.
 *
 * @return Always returns 1
 */
__attribute__((weak)) int _getpid(void) {
    return 1;
}

/**
 * @brief Get file status (stub for fstat).
 * 
 * This function provides information about a file. Always returns a character device.
 *
 * @param file File descriptor (ignored)
 * @param st Pointer to stat structure to fill
 * @return Always returns 0
 */
__attribute__((weak)) int _fstat(int file, struct stat *st) {
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

/**
 * @brief Close a file (stub for close).
 * 
 * Since there is no filesystem, this function does nothing.
 *
 * @param file File descriptor (ignored)
 * @return Always returns -1
 */
__attribute__((weak)) int _close(int file) {
    (void)file;
    return -1;
}
