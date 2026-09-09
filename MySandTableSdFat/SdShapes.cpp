/******************************************************************************
 * File: SdShapes.cpp
 *
 * This file implements the SdShapes class.  This class manages the files on an
 * SD card.  It is used to initialize access to the SD card, and contains methods
 * to open random and specific shape files which already reside on the SD card.
 *
 * History:
 * - 09-SEP-2026 Joe Corbett (JMC)
 *   - Original work.
 *
 * Copyright (c) 2026, Joseph M. Corbett
 *******************************************************************************/
#include "SdShapes.h"


/******************************************************************************
 * Constructor
 *******************************************************************************/
SdShapes::SdShapes() : m_pRootDir(NULL), m_pRng(NULL), m_Sd(), m_SdInitd(false),
             m_Root(), m_RootOpen(false), m_NumShapes(0), m_File(),
             m_FileIsOpen(false), m_FileName(0)
             {}


/******************************************************************************
* Destructor
*******************************************************************************/
SdShapes::~SdShapes()
{
    CloseFile();
    m_Root.close();
    m_Sd.end();
} // End destructor.


/*******************************************************************************
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
*   either the SD is not initialized successfully or if the root cannot be setup.
*******************************************************************************/
bool SdShapes::Begin(pRNG_t pRng, const char *pRoot, SdCsPin_t csPin,
                     uint8_t opt, uint32_t spd, SpiPort_t *port)
{
    // Close any file that may have already been open.
    CloseFile();

    // Initialize our data.
    m_pRng = pRng;
    m_pRootDir = pRoot;
    m_FileName[0] = '\0';

    // The SD card may have been previously initialized.  If so, terminate it.
    if (m_SdInitd)
    {
        m_Sd.end();
    }

    // Start the SD hardware.
    m_SdInitd = m_Sd.begin(SdSpiConfig(csPin, opt, spd, port));

    // Setup the root directory.
    SetRootDir(pRoot);

    // Return status
    return m_SdInitd && m_RootOpen;
} // End Begin().


/*******************************************************************************
* SetRootDir()
*
* Sets a (possibly) new root directory in which to find the shape files.
*
* Arguments:
*   - pDir : String containing the path to the (possibly) new root directory.
*
* Returns:
*   Returns 'true' if successful, 'false' otherwise.  May return 'false' if
*   either the SD is not initialized successfully or if the root cannot be setup.
*******************************************************************************/
bool SdShapes::SetRootDir(const char *pDir)
{
    // Cleanup the (possibly) already open root directory.
    CloseFile();
    m_RootOpen = false;
    m_pRootDir = pDir;

    // Don't bother if the SD is not initialized.
    if (m_SdInitd)
    {
        // Close the (possibly) existing root, and initialize data.
        m_Root.close();
        m_NumShapes = 0;

        // Attempt to open the new root.
        m_RootOpen = m_Root.open(m_pRootDir);
        if (m_RootOpen)
        {
            // Count the number of shape files.
            FsFile myFile;
            m_Root.rewind();
            while (myFile.openNext(&m_Root, O_RDONLY))
            {
                // Be sure to count only valid files.
                if (!myFile.isHidden() && myFile.isFile() && myFile.size())
                {
                    m_NumShapes++;
                }
                myFile.close();
            }
        }
    }

    // Return our status.
    return m_RootOpen;
} // End SetRootDir().


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
bool &SdShapes::RandomShapeFile(oflag_t flag)
{
    // Close any file that may already be open.
    CloseFile();

    // Don't bother unless there are files to select from.
    if (m_NumShapes)
    {
        // Generate a random index into our shapes.
        uint32_t indx = (*m_pRng)(1, m_NumShapes + 1);

        // Loop through existing files until the selected random one is found.
        m_Root.rewindDirectory();
        for (uint32_t i = 0; i < indx; i++)
        {
            // Don't try to close any hidden files.
            if (!m_File.isHidden())
            {
                // Make sure we only have 1 file open at a time.
                m_File.close();
            }

            // Only count files that contain shapes.
            m_FileIsOpen = m_File.openNext(&m_Root, flag);
            if (m_File.isHidden() || !m_File.isFile() || !m_File.size())
            {
                i--;
            }
        }

        // We found a file, save its name.
        m_File.getName(m_FileName, sizeof(m_FileName));
    }

    // Return an indication of our success or failure.
    return m_FileIsOpen;
} // End RandomShapeFile().


/*******************************************************************************
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
*******************************************************************************/
bool SdShapes::OpenFile(const char *pName, oflag_t flag)
{
    // Close any file that may have already been open.
    CloseFile();

    // Attempt to open the file.
    m_FileIsOpen = m_File.open(pName, flag);
    if (m_FileIsOpen)
    {
        // If we succeeded then save the file name.
        m_File.getName(m_FileName, sizeof(m_FileName));
    }

    // Return an indication of our success or failure.
    return m_FileIsOpen;
} // End OpenFile().


/*******************************************************************************
* CloseFile()
*
* Closes the currently open file (if any).
*
* Returns:
*   Returns 'true' if successful, 'false' if the file was already closed.
*******************************************************************************/
bool SdShapes::CloseFile()
{
    // Reset our data, then  close the file.
    m_FileName[0] = '\0';
    m_FileIsOpen = false;
    return m_File.close();
} // End CloseFile().

