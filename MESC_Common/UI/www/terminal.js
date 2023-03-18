/*
* Copyright 2023 cod3b453
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

function terminal_init()
{
    var obj_app = document.getElementById( 'app' );

    var obj_history = document.createElement( 'div' );

        var obj_text = document.createElement( 'textarea' );
        obj_text.id = 'app-terminal-history'
        obj_text.readOnly = true;
        obj_history.appendChild( obj_text );

    obj_app.appendChild( obj_history );

    var obj_form = document.createElement( 'form' );

    obj_form.method = 'GET';
    obj_form.action = 'UART';

        var obj_input = document.createElement( 'input' );
        obj_input.type = 'text';
        obj_input.name = 'command';

    obj_form.appendChild( obj_input );

        var obj_button = document.createElement( 'input' );
        obj_button.type = 'submit';
        obj_button.value = 'Send';

    obj_form.appendChild( obj_button );

    obj_app.appendChild( obj_form );

    obj_app.innerHTML += "Terminal loaded!";
}

console.log( "Starting Terminal module" );
terminal_init();
console.log( "Finished Terminal module" );
