/*
 *****************************************************************************
 * Copyright @ 2010 by austriamicrosystems AG                                *
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
 *      PROJECT:   ASxxxx firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file errno.h
 *
 *  \author M. Arpa
 *
 *  \brief Main error codes. Please add your application specific 
 *  error codes in your application starting with
 *  #define MY_APP_ERR_CODE (ERR_LAST_ERROR-1)
 *
 */

#ifndef ERRNO_H
#define ERRNO_H

/*! \defgroup errorcodes Basic application error codes
 * Error codes to be used within the application.
 * They are represented by an s8
 */
#define ERR_NONE   0 /*!< \ingroup errorcodes
               no error occured */
#define ERR_NOMEM -1 /*!< \ingroup errorcodes
               not enough memory to perform the requested
               operation */
#define ERR_BUSY  -2 /*!< \ingroup errorcodes
               device or resource busy */
#define ERR_IO    -3 /*!< \ingroup errorcodes
                generic IO error */
#define ERR_TIMEOUT -4 /*!< \ingroup errorcodes
                error due to timeout */
#define ERR_REQUEST -5 /*!< \ingroup errorcodes
                invalid request or requested
                function can't be executed at the moment */
#define ERR_NOMSG -6 /*!< \ingroup errorcodes
                       No message of desired type */
#define ERR_PARAM -7 /*!< \ingroup errorcodes
                       Parameter error */

#define ERR_LAST_ERROR -32

#endif /* ERRNO_H */

