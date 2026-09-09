/******************************************************************************
 * File: SdShapes.h
 *
 * This file defines the SdShapes class.  This class manages the files on an SD
 * card.  It is used to initialize access to the SD card, and contains methods
 * to open random and specific shape files which already reside on the SD card.
 *
 * History:
 * - 09-SEP-2026 Joe Corbett (JMC)
 *   - Original work.
 *
 * Copyright (c) 2026, Joseph M. Corbett
 *******************************************************************************/
#if !defined SDSHAPES_H
#define SDSHAPES_H

#include "SdFat.h"

// Use a typedef for a pointer to the random function to make its use easier.
typedef int32_t (*pRNG_t)(int32_t, int32_t);


/******************************************************************************
 * SdShapes class
 *
 * This class manages the files on an SD card.  It is used to initialize access
 * to the SD card, and contains methods to open random and specific shape files
 * which already reside on the SD card.
 *
 * Note that some methods will open a file for the specified type of access, but
 * the file should only be closed by the CloseFile() method.  Since under
 * SdFs, only one file may be open at a time, this class takes care of closing
 * any open file before opening another, but the CloseFile() method is included
 * for special application needs.
 *******************************************************************************/
class SdShapes
{
public:
    /***************************************************************************
    * constructor & destructor
    ***************************************************************************/
    SdShapes();
    ~SdShapes();

    /***************************************************************************
    * Begin()
    *
    * Initializes everythingincluding:
    *   - The SD card hardware.
    *   - The file system.
    *   - The root directory.
    *   - The random file generation logic.
    *
    * Arguments:
    *   - pRng  : Pointer to the random number generation function.
    *   - pRoot : Pointer to a string describing the root directory path.
    *   - csPin : SD card chip select pin.
    *   - opt   : Selects SPI bus config:
    *             DEDICATED_SPI - indicates that the SPI bus us used exclusively
    *                             by the SD card.  It allows optimizations by
    *                             allowing optimizations by skipping bus-sharing
    *                             checks and state restorations.
    *             SHARED_SPI    - indicates that the SD card shares the SPI bus
    *                             with other devices, and precludes certain
    *                             optimizations.
    *   - spd   : Max SPI bus speed selection:
    *             SD_SCK_HZ()         - Select SPI clock frequency in Hz.
    *             SD_CHK_MHZ()        - Select SPI clock frequency in MHz.
    *             SPI_FULL_SPEED      - Same as SD_SCK_MHZ(50).
    *             SPI_HALF_SPEED      - Same as SD_SCK_MHZ(4).
    *             SPI_QUARTER_SPEED   - Same as  SD_SCK_MHZ(2).
    *             SPI_SIXTEENTH_SPEED - Same as SD_SCK_HZ(500000).
    *   - port  : The SPI port to use.
    *
    * Returns:
    *   Returns 'true' if successful, 'false' otherwise.  May return 'false' if
    *   either the SD is not initialized successfully or if the root cannot be
    *   setup.
    ***************************************************************************/
    bool Begin(pRNG_t pRng, const char *pRoot, SdCsPin_t csPin,
               uint8_t opt = DEDICATED_SPI, uint32_t spd = SPI_FULL_SPEED,
               SpiPort_t *port = &SPI1);


    /***************************************************************************
    * SetRootDir()
    *
    * Sets a (possibly) new root directory in which to find the shape files.
    *
    * Arguments:
    *   - pDir : String containing the path to the (possibly) new root directory.
    *
    * Returns:
    *   Returns 'true' if successful, 'false' otherwise.  May return 'false' if
    *   either the SD is not initialized successfully or if the root cannot be
    *   setup.
    ***************************************************************************/
    bool SetRootDir(const char *pDir);


    /***************************************************************************
    * RandomShapeFile()
    *
    * Opens a random shape file.
    *
    * Arguments:
    *   - flag : File open type:
    *            O_RDONLY - Open for reading.
    *            O_READ   - Same as O_RDONLY.
    *            O_WRONLY - Open for writing.
    *            O_WRITE  - Same as O_WRONLY.
    *            O_RDWR   - Open for reading and writing.
    *            O_APPEND - If set, the file offset shall be set to the end of
    *                       the file prior to each write.
    *            O_AT_END - Set the initial position at the end of the file.
    *            O_CREAT  - If the file exists, this flag has no effect except
    *                       as noted under O_EXCL below. Otherwise, the file
    *                       shall be created
    *            O_EXCL   - If O_CREAT and O_EXCL are set, open() shall fail if
    *                       the file exists.
    *            O_TRUNC  - If the file exists and is a regular file, and the
    *                       file is successfully opened and is not read only,
    *                       its length shall be truncated to 0.
    *
    * Returns:
    *   Returns 'true' if a random shape file was successfully opened, 'false'
    *   otherwise.  If the file was successfully opened, then the file name may
    *   be retrieved via a call to  CurFileName(), and a reference to the file
    *   may be retrieved via a call to CurFile().
    ***************************************************************************/
    bool &RandomShapeFile(oflag_t flag = O_RDONLY);


    /***************************************************************************
    * OpenFile()
    *
    * Opens the specified file for the specified type of access.
    *
    * Arguments:
    *   - pName: String containing the name of the file to be opened.
    *   - flag : File open type:
    *            O_RDONLY - Open for reading.
    *            O_READ   - Same as O_RDONLY.
    *            O_WRONLY - Open for writing.
    *            O_WRITE  - Same as O_WRONLY.
    *            O_RDWR   - Open for reading and writing.
    *            O_APPEND - If set, the file offset shall be set to the end of
    *                       the file prior to each write.
    *            O_AT_END - Set the initial position at the end of the file.
    *            O_CREAT  - If the file exists, this flag has no effect except
    *                       as noted under O_EXCL below. Otherwise, the file
    *                       shall be created
    *            O_EXCL   - If O_CREAT and O_EXCL are set, open() shall fail if
    *                       the file exists.
    *            O_TRUNC  - If the file exists and is a regular file, and the
    *                       file is successfully opened and is not read only,
    *                       its length shall be truncated to 0.
    *
    * Returns:
    *   Returns 'true' if the specified file was successfully opened, 'false'
    *   otherwise.  If the file was successfully opened, then the file name may
    *   be retrieved via a call to  CurFileName(), and a reference to the file
    *   may be retrieved via a call to CurFile().
    ***************************************************************************/
    bool OpenFile(const char *pName, oflag_t flag = O_RDONLY);


    /***************************************************************************
    * CloseFile()
    *
    * Closes the currently open file (if any).
    *
    * Returns:
    *   Returns 'true' if successful, 'false' if the file was already closed.
    ***************************************************************************/
    bool CloseFile();


    /***************************************************************************
    * Simple getters.
    ***************************************************************************/
    uint32_t    NumShapes()  const { return m_NumShapes;  }
    bool        IsSdInitd()  const { return m_SdInitd;    }
    bool        IsRootOpen() const { return m_RootOpen;   }
    const char *RootDir()    const { return m_pRootDir;   }
    bool        IsFileOpen() const { return m_FileIsOpen; }
    const char *CurFileName()      { return m_FileName;   }
    FsFile     &CurFile()          { return m_File;       }


    // The maximum file name length.
    static const size_t MAX_NAME = 64;

private:
    const char *m_pRootDir;     // Root directory path string.
    pRNG_t      m_pRng;         // Pointer to random generator function.
    SdFs        m_Sd;           // SD card interface.
    bool        m_SdInitd;      // True if the SD has been initialized.
    FsFile      m_Root;         // Root directory file.
    bool        m_RootOpen;     // True if root dir opened successfully.
    uint32_t    m_NumShapes;    // Number of shape files found.
    FsFile      m_File;         // Currently open file.
    bool        m_FileIsOpen;   // True if m_File is open.
    char        m_FileName[MAX_NAME];    // Name string of current file.
}; // End class SdShapes.

#endif // SDSHAPES_H