/* $Id: capifs.h,v 1.1.1.1 2005/03/24 06:16:44 arch-linux Exp $
 * 
 * Copyright 2000 by Carsten Paeth <calle@calle.de>
 *
 * This software may be used and distributed according to the terms
 * of the GNU General Public License, incorporated herein by reference.
 *
 */

void capifs_new_ncci(unsigned int num, dev_t device);
void capifs_free_ncci(unsigned int num);
