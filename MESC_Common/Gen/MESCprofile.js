/*
* Copyright 2021 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

const PROFILE_SIGNATURE = 'MESC';

const PROFILE_VERSION_MAJOR = 1;
const PROFILE_VERSION_MINOR = 0;

const PROFILE_MAX_ENTRIES = 8;

const PROFILE_HEADER_SIZE = 64;

// ProfileEntryMap
const PROFILE_ENTRY_R = 1;
const PROFILE_ENTRY_W = 2;

const PROFILE_ENTRY_RW = (PROFILE_ENTRY_R | PROFILE_ENTRY_W);
// Aliases
const PROFILE_ENTRY_DATA = 0;
const PROFILE_ENTRY_LOCK = PROFILE_ENTRY_R;
const PROFILE_ENTRY_FREE = PROFILE_ENTRY_W;
const PROFILE_ENTRY_USED = PROFILE_ENTRY_RW;
//end

const PROFILE_ENTRY_BITS = 2;
const PROFILE_ENTRY_MASK = 3;

const PROFILE_HEADER_ENTRIES = ((BITS_PER_BYTE * /*sizeof(ProfileHeader::entry_map)*/PROFILE_MAX_ENTRIES) / PROFILE_ENTRY_BITS);

const PROFILE_ENTRY_SIGNATURE = 'MPEH';

const PROFILE_ENTRY_MAX_NAME_LENGTH = 12;

const ENTRIES_PER_MAP_ENTRY = (PROFILE_MAX_ENTRIES / PROFILE_ENTRY_BITS);

// ProfileHeader

function ProfileHeader()
{
    this._image = []; //ArrayBuffer?

    this._entry    = new Array( PROFILE_MAX_ENTRIES * ENTRIES_PER_MAP_ENTRY );
    this.entry_map = new Array( PROFILE_MAX_ENTRIES );
}

ProfileHeader.prototype.addEntry = function(index,signature)
{
    var entry_map_index  = index / ENTRIES_PER_MAP_ENTRY;
    var entry_map_offset = index % ENTRIES_PER_MAP_ENTRY;

    this.entry_map[entry_map_index] = (this.entry_map[entry_map_index] & ~(PROFILE_ENTRY_MASK << (entry_map_offset * PROFILE_ENTRY_BITS)))
                                    |                                     (PROFILE_ENTRY_USED << (entry_map_offset * PROFILE_ENTRY_BITS));

    this._entry[index] = new ProfileEntry(this,index,signature);
    return this.getEntry(index);
};

ProfileHeader.prototype.getEntry = function(index)
{
    return this._entry[index];
}

ProfileHeader.prototype.dump_image = function()
{
    var hex = '';

    return hex;
};

ProfileHeader.prototype.checksum = function()
{
    var fnv = fnv1a_init();

    // TODO fnv1a_process( fnv, byte )

    return fnv;
};

ProfileHeader.prototype.image_length = function()
{
    // TODO
    return 0;
};

ProfileHeader.prototype.image_checksum = function()
{
    var fnv = fnv1a_init();

    // TODO fnv1a_process( fnv, byte )

    return fnv;
};

function dump_ProfileHeader( header )
{
    var hex = '';

    hex = hex + dump_c_char( PROFILE_SIGNATURE, 4 );

    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_uint8_t( PROFILE_VERSION_MAJOR );
    hex = hex + dump_c_uint8_t( PROFILE_VERSION_MINOR );
    hex = hex + dump_c_uint8_t( PROFILE_HEADER_SIZE   );

    hex = hex + dump_c_uint32_t( header.checksum() );

    for ( let e = 0; e < PROFILE_MAX_ENTRIES; e++ )
    {
        hex = hex + dump_c_uint8_t( header.entry_map[e] );
    }

    hex = hex + dump_c_uint32_t( header.image_length()   );
    hex = hex + dump_c_uint32_t( header.image_checksum() );

    hex = hex + dump_MESCFingerprint();

    hex = hex + header.dump_image();

    return hex;
}

function ProfileEntry( header, index, signature )
{
    this._header = header;
    this._index = index;

    this._name = '';

    this._data_signature = signature;

    this._data_length = 0;
    this._data_offset= 0;
}

ProfileEntry.prototype.setName = function( name )
{

}

// ProfileEntry
function dump_ProfileEntry( entry )
{
    var hex = '';

    hex = hex + dump_c_char( PROFILE_ENTRY_SIGNATURE, 4 );

    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_uint8_t( PROFILE_ENTRY_SIZE );
    hex = hex + dump_c_uint8_t( entry.name.length );
    hex = hex + dump_c_char( entry.name, PROFILE_ENTRY_MAX_NAME_LENGTH );
    hex = hex + dump_c_uint8_t( 0 );

    hex = hex + dump_c_uint32_t( entry.data_signature );

    hex = hex + dump_c_uint32_t( entry.data_length );
    hex = hex + dump_c_uint32_t( entry.data_offset );

    return hex;
}
