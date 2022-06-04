/*
* Copyright 2021-2022 cod3b453
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

const PROFILE_ENTRY_SIZE = 32;

const ENTRIES_PER_MAP_ENTRY = (PROFILE_MAX_ENTRIES / PROFILE_ENTRY_BITS);

// ProfileHeader

function ProfileHeader()
{
    this._image = []; //ArrayBuffer?

    this._entry    = new Array( PROFILE_MAX_ENTRIES * ENTRIES_PER_MAP_ENTRY );
    this.entry_map = new Array( PROFILE_MAX_ENTRIES );

    for ( let e = 0, i = 0; e < PROFILE_MAX_ENTRIES; e++ )
    {
        var v = 0;

        for ( let me = 0; me < ENTRIES_PER_MAP_ENTRY; me++, i++ )
        {
            this._entry[i] = new ProfileEntry( this, i, 'NULL' );
            v = (v << PROFILE_ENTRY_BITS) | PROFILE_ENTRY_FREE;
        }

        this.entry_map[e] = v;
    }
}

ProfileHeader.prototype.addEntry = function(index,signature)
{
    var entry_map_index  = Math.floor(index / ENTRIES_PER_MAP_ENTRY);
    var entry_map_offset =            index % ENTRIES_PER_MAP_ENTRY;

    this.entry_map[entry_map_index] = (this.entry_map[entry_map_index] & ~(PROFILE_ENTRY_MASK << (entry_map_offset * PROFILE_ENTRY_BITS)))
                                    |                                     (PROFILE_ENTRY_USED << (entry_map_offset * PROFILE_ENTRY_BITS));

    this._entry[index] = new ProfileEntry( this, index, signature );
    return this.getEntry(index);
};

ProfileHeader.prototype.getEntry = function(index)
{
    return this._entry[index];
}

ProfileHeader.prototype.remEntry = function(index)
{
    var entry_map_index  = Math.floor(index / ENTRIES_PER_MAP_ENTRY);
    var entry_map_offset =            index % ENTRIES_PER_MAP_ENTRY;

    this.entry_map[entry_map_index] = (this.entry_map[entry_map_index] & ~(PROFILE_ENTRY_MASK << (entry_map_offset * PROFILE_ENTRY_BITS)))
                                    |                                     (PROFILE_ENTRY_FREE << (entry_map_offset * PROFILE_ENTRY_BITS));

    return this._entry[index] = new ProfileEntry( this, index, 'NULL' );
}

ProfileHeader.prototype.dump_image = function()
{
    var hex = '';
    var offset = PROFILE_HEADER_SIZE + (PROFILE_HEADER_ENTRIES * PROFILE_ENTRY_SIZE);

    for ( let e = 0, i = 0; e < PROFILE_MAX_ENTRIES; e++ )
    {
        for ( let me = 0; me < ENTRIES_PER_MAP_ENTRY; me++, i++ )
        {
            var pe = (this.entry_map[e] >> (PROFILE_ENTRY_BITS * me)) & PROFILE_ENTRY_MASK;

            switch (pe) {
                case PROFILE_ENTRY_FREE:
                    // Nothing
                    hex = hex + dump_c_char( '', PROFILE_ENTRY_SIZE );
                    break;
                case PROFILE_ENTRY_DATA:
                case PROFILE_ENTRY_LOCK:
                case PROFILE_ENTRY_USED:
                    this._entry[i]._data_length = this._entry[i]._profile.size();
                    this._entry[i]._data_offset = offset;

                    offset = offset + this._entry[i]._data_length;

                    console.log( "dump_ProfileEntry[" + i.toString() + "]" );
                    hex = hex + dump_ProfileEntry( this._entry[i] );
                    break;
            }
        }
    }

    for ( let e = 0, i = 0; e < PROFILE_MAX_ENTRIES; e++ )
    {
        for ( let me = 0; me < ENTRIES_PER_MAP_ENTRY; me++, i++ )
        {
            var pe = (this.entry_map[e] >> (PROFILE_ENTRY_BITS * me)) & PROFILE_ENTRY_MASK;

            switch (pe) {
                case PROFILE_ENTRY_FREE:
                    // Nothing
                    break;
                case PROFILE_ENTRY_DATA:
                case PROFILE_ENTRY_LOCK:
                case PROFILE_ENTRY_USED:
                    hex = hex + this._entry[i]._profile.dump();
                    break;
            }
        }
    }

    return hex;
};

function dump_ProfileHeader( header )
{
    console.log( "dump_ProfileHeader" );
    var hex = '';

    hex = hex + dump_c_char( PROFILE_SIGNATURE, 4 );

    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_uint8_t( PROFILE_VERSION_MAJOR );
    hex = hex + dump_c_uint8_t( PROFILE_VERSION_MINOR );
    hex = hex + dump_c_uint8_t( PROFILE_HEADER_SIZE   );

    var offsetof_checksum = hex.length;
    hex = hex + dump_c_char( PROFILE_SIGNATURE, 4 ); // checksum
    var endof_checksum = hex.length;

    for ( let e = 0; e < PROFILE_MAX_ENTRIES; e++ )
    {
        hex = hex + dump_c_uint8_t( header.entry_map[e] );
    }

    var image_hex = header.dump_image();

    var image_length = image_hex.length / NYBBLES_PER_BYTE;
    var image_checksum = fnv1a_process_hex( image_hex );

    console.log( "image_length   " + image_length.toString() );
    console.log( "image_checksum " + image_checksum.toString(16).toUpperCase() );

    hex = hex + dump_c_uint32_t( image_length   );
    hex = hex + dump_c_uint32_t( image_checksum );

    hex = hex + dump_MESCFingerprint();

    console.assert( hex.length == (NYBBLES_PER_BYTE * PROFILE_HEADER_SIZE) );

    var checksum = fnv1a_process_hex( hex );

    console.log( "PATCH " + hex );

    hex = hex.substring(0,offsetof_checksum) + dump_c_uint32_t(checksum) + hex.substring(endof_checksum);

    console.log( "PATCH " + hex );

    hex = hex + image_hex;

    return hex;
}

function ProfileEntry( header, index, signature )
{
    this._header = header;
    this._index = index;

    this._name = undefined;

    this._data_signature = signature;

    this._data_length = undefined;
    this._data_offset = undefined;
}

ProfileEntry.prototype.setName = function( name )
{
    this._name = name;
}

// ProfileEntry
function dump_ProfileEntry( entry )
{
    var hex = '';

    hex = hex + dump_c_char( PROFILE_ENTRY_SIGNATURE, 4 );

    hex = hex + dump_c_uint8_t( 0 );
    hex = hex + dump_c_uint8_t( PROFILE_ENTRY_SIZE );
    hex = hex + dump_c_uint8_t( entry._name.length );
    hex = hex + dump_c_char( entry._name, PROFILE_ENTRY_MAX_NAME_LENGTH );
    hex = hex + dump_c_uint8_t( 0 );

    hex = hex + dump_c_uint32_t( entry._data_signature );

    hex = hex + dump_c_uint32_t( entry._data_length );
    hex = hex + dump_c_uint32_t( entry._data_offset );

    console.assert( hex.length == (NYBBLES_PER_BYTE * PROFILE_ENTRY_SIZE) );

    return hex;
}
