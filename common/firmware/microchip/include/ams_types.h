/*
 *****************************************************************************
 * Copyright @ 2009 by austriamicrosystems AG                                *
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

/*! \file ams_types.h
 *
 *  \author 
 *
 *  \brief Basic datatypes
 *  
 */

#ifndef AMS_TYPES_H
#define AMS_TYPES_H

#include "GenericTypeDefs.h"

/*! \defgroup datatypes Basic datatypes
 * Basic datatypes are mapped to austriamicrosystems datatypes that
 * shall be used in all austriamicrosystems projects.
 */
typedef unsigned char u8;   /*!< \ingroup datatypes
							  represents an unsigned 8bit-wide type */
typedef signed char s8;     /*!< \ingroup datatypes
							  represents a signed 8bit-wide type */
typedef unsigned int u16;   /*!< \ingroup datatypes
							  represents an unsigned 16bit-wide type */
typedef signed int s16;     /*!< \ingroup datatypes
							  represents a signed 16bit-wide type */
typedef unsigned long u32;  /*!< \ingroup datatypes
							  represents an unsigned 32bit-wide type */
typedef unsigned long long u64;  /*!< \ingroup datatypes
								   represents an unsigned 64bit-wide type */
typedef signed long s32;      /*!< \ingroup datatypes
								represents a signed 32bit-wide type */
typedef u16 umword; /*!< \ingroup datatypes
                        USE WITH CARE!!! unsigned machine word:
                        8 bit on 8bit machines, 16 bit on 16 bit machines... */
typedef s16 smword; /*!< \ingroup datatypes
                        USE WITH CARE!!! signed machine word:
                         8 bit on 8bit machines, 16 bit on 16 bit machines... */
typedef unsigned int uint; /*!< \ingroup datatypes
                            type for unsigned integer types,
                            useful as indices for arrays and loop variables */
typedef signed int sint; /*!< \ingroup datatypes
                            type for integer types, 
                            useful as indices for arrays and loop variables */

#define U8_C(x)     (x) /*!< \ingroup datatypes
                         Define a constant of type u8 */
#define S8_C(x)     (x) /*!< \ingroup datatypes
                         Define a constant of type s8 */
#define U16_C(x)    (x) /*!< \ingroup datatypes
                         Define a constant of type u16 */
#define S16_C(x)    (x) /*!< \ingroup datatypes
                         Define a constant of type s16 */
#define U32_C(x)    (x##UL) /*!< \ingroup datatypes
                             Define a constant of type u32 */
#define S32_C(x)    (x##L) /*!< \ingroup datatypes
                            Define a constant of type s32 */
#define U64_C(x)    (x##ULL) /*!< \ingroup datatypes
                              Define a constant of type u64 */
#define S64_C(x)    (x##LL) /*!< \ingroup datatypes
                             Define a constant of type s64 */
#define UMWORD_C(x) (x) /*!< \ingroup datatypes
                         Define a constant of type umword */
#define MWORD_C(x)  (x) /*!< \ingroup datatypes
                         Define a constant of type mword */

#if 0
typedef umword bool_t; /*!< \ingroup datatypes
                            represents a boolean type */

#ifndef TRUE
#define TRUE 1 /*!< \ingroup datatypes
		 used for the #bool_t type */
#endif
#ifndef FALSE
#define FALSE 0 /*!< \ingroup datatypes
		 used for the #bool_t type */
#endif

#else 
typedef BOOL bool_t;
#endif


#ifndef NULL
#define NULL (void*)0 /*!< \ingroup datatypes
		 represents a NULL pointer */
#endif

#endif /* AMS_TYPES_H */

