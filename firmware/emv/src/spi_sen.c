/*
 *****************************************************************************
 * Copyright @ 2011 by austriamicrosystems AG                                *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file main.c
 *
 *  \author Oliver Regenfelder
 *
 *  \brief main application file
 *
 *  This is the implementation file for the main loop
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <p24FJ64GB002.h>

#include "ams_types.h"

#include "spi_sen.h"

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION
******************************************************************************
*/

/*!
 *****************************************************************************
 * \brief Select the SPI target chip.
 *
 * Select the chip connected to the SPI which shall be the target of all
 * subsequent SPI1 operations.
 *
 * \param newSpiTarget Select the SPI target slave chip.
 *****************************************************************************
 */
SPIChip_t spiSelectChip(SPIChip_t newSpiTarget)
{
    SPIChip_t previouslySelectedChip = spiGetSelectedChip();

    if (newSpiTarget == SPI_CHIP_AS3910_RX)
    {
        _LATA0 = 1;
        _LATA1 = 0;
    }
    else if (newSpiTarget == SPI_CHIP_AS3910_TX)
    {
        _LATA0 = 0;
        _LATA1 = 0;
    }
    else if (newSpiTarget == SPI_CHIP_XILINX)
    {
        _LATA0 = 0;
        _LATA1 = 1;
    }
    else
    {
        // Handle error here
    }

    return previouslySelectedChip;
}

/*!
 *****************************************************************************
 * \brief Get the currently selected SPI target chip.
 *
 * \return The chip currently selected as SPI slave.
 *****************************************************************************
 */
SPIChip_t spiGetSelectedChip()
{
    if (1 == _LATA1)
        return SPI_CHIP_XILINX;
    else if (0 == _LATA0)
        return SPI_CHIP_AS3910_TX;
    else
        return SPI_CHIP_AS3910_RX;
}
