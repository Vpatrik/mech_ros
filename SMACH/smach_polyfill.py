"""
# Copyright (c) 2018, Sebastian Putz
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

from smach import CBInterface as SmachCBInterface
from smach import cb_interface as smach_cb_interface


class CBInterface(SmachCBInterface):
    """
    Support for decorated methods (instead of pure functions) as callbacks. Takes care of binding the instance
    to this object and passing it to the method as the self parameter.
    """
    def __init__(self, cb, outcomes=[], input_keys=[], output_keys=[],
                 io_keys=[], _instance=None):
        """Save ths isntance"""
        SmachCBInterface.__init__(self, cb, outcomes, input_keys, output_keys, io_keys)
        self._instance = _instance

    def __get__(self, obj, type=None):
        """Caputes the instance. Returns a new object with the instance assigned."""
        return CBInterface(self._cb, outcomes=self._outcomes, input_keys=self._input_keys, output_keys=self._output_keys, _instance=obj)

    def __call__(self, *args, **kwargs):
        """Executes the callback. If an instance is given, it will be passed for the this parameter"""
        if self._instance is not None:
            return self._cb(self._instance, *args, **kwargs)
        else:
            return self._cb(*args, **kwargs)

class cb_interface(smach_cb_interface):
    """Modified interface, so the new CBInterface from this polyfill will be used"""
    def __init__(self, *args, **kwargs):
        smach_cb_interface.__init__(self, *args, **kwargs)

    def __call__(self, cb):
        return CBInterface(cb, self._outcomes, self._input_keys, self._output_keys)