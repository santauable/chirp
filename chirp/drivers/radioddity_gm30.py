# Copyright 2025 Mike Iacovacci <ascendr@linuxmail.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import struct
import logging

from chirp import chirp_common, directory, memmap
from chirp import bitwise, errors
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueInteger, RadioSettingValueList, \
    RadioSettingValueBoolean, RadioSettingValueString, RadioSettings

LOG = logging.getLogger(__name__)

MEM_FORMAT = """
struct{
    u8     unknown02[256];
} unknown02[16];
struct{
    u8      bootscrmode;
    u8      bsmodepad[15];
    u8      bootscreen1[10];
    u8      bs1pad[6];
    u8      bootscreen2[10];
    u8      bs2pad[6];
    u8      unused[16];
    u8      timeout;
    u8      squelch;
    u8      vox_level;
    u8      batt_save:4,
            unk_bits:2,
            work_mode:1,
            voice_alert:1;
    u8      backlight;
    u8      beep_tone:1,
            auto_key_lock:1,
            unk_bit_2:1,
            ctcss_revert:1,
            scan_type:2,
            side_tone:2;
    u8      unk_bit_3:1,
            standby:1,
            roger:1,
            alarm_mode:2,
            alarm_sound:1,
            fm_radio:1,
            unk_bit_4:1;
    u8      tail_revert;
    u8      tail_delay;
    u8      tbst;
    u8      unk_sett[6];
    u8      unk_bits_5:6,
            b_ch_disp:1,
            a_ch_disp:1;
    u8      unk_sett_2[24];
    u8      passw_w_ena;
    u8      passw_r_ena;
    u8      passw_w_val[8];
    u8      passw_r_val[8];
    u8      unk_sett_3[5];
} settings;
struct{
    u8     unusedsettings[32];
} unusedsettings[124];
struct{
    u8     entry[5];
} dtmf_list[15];
struct{
    u8      dtmf_l_pad[5];
    u8      radio_id[5];
    u8      unk_dtmf[11];
    u8      unk_dtmf_bits:6,
            press_send:1,
            release_send:1;
    u8      delay_time;
    u8      digit_dur;
    u8      inter_dur;
    u8      unk_dtmf_2[28];
} dtmf;
struct{
    u8     unkdtmf[128];
} unuseddtmf[31];
struct{
    u8        unk_mem[16];
} tuning;
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} vfomemory[2];
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} memory[250];
struct{
    u8      unused_memch[16];
} unused_mem[3];
struct{
    u8     unknown17[256];
} unknown17[16];
struct{
    u8     unknown18[256];
} unknown18[16];
struct{
    u8     unknown19[256];
} unknown19[16];
struct{
    char      name[6];
    u8      pad[5];
} memnames[250];
struct{
    u8      unknames[64];
} unknownnames [21];
struct{
    u8      unknown25[256];
} unknown25[16];
struct{
    u8      unknown26[256];
} unknown26[16];
"""
# radio defines these types
# query each type to retrieve
# the mem location it is stored
TYPE_MAP = [
    (0x02, "unknown02"),
    (0x04, "settings"),
    (0x06, "dtmf"),
    (0x16, "memories"),
    (0x17, "unknown17"),
    (0x18, "unknown18"),
    (0x19, "unknown19"),
    (0x24, "chan_names"),
    (0x25, "unknown25"),
    (0x26, "unknown26")
]

# we only write these data types
# back to the radio. The 3rd tuple
# being the offset in _memobj
WRITE_MAP = [
    (0x04, "settings", 0x1000),
    (0x06, "dtmf", 0x2000),
    (0x16, "memories", 0x3000),
    (0x24, "chan_names", 0x7000)
]


def do_ack_ack(serial):
    serial.write(b'\x06')
    ack = serial.read(1)
    if ack != b'\x06':
        err = f"Error expected 06 ack got {ack}"
        LOG.debug(err)
        raise errors.RadioError(err)


def raw_send(serial, data, exlen):
    serial.write(data)
    return serial.read(exlen)


def do_read_cmd(serial, cmd, exlen):
    echo_ack = len(cmd) + 1  # ack 0x57 + echo of cmd
    resp = raw_send(serial, b'\x52' + cmd, exlen + echo_ack)
    if not resp or resp[0] != 0x57:
        raise errors.RadioError(f"Read CMD resp failed got {resp}")
    if len(resp[echo_ack:]) != exlen:
        raise errors.RadioError(f"Read CMD resp expect len={exlen} got {resp}")
    do_ack_ack(serial)
    return resp[echo_ack:]


def enter_prog(serial):
    resp = raw_send(serial, b'PSEARCH', 8)
    if len(resp) != 8:
        err = ("Enter programming failed" +
               f" Resp : {resp}")
        raise errors.RadioError(err)
    if resp[0] != 0x06:
        raise errors.RadioError("Enter programming: " +
                                f"Radio Bad Ack: {resp}")
    return resp[1:]


def exit_prog(serial):
    try:
        serial.write(b'\x06')
        serial.write(b'\x06')
        serial.write(b'\x00')
        serial.close()
        LOG.debug("Exited programming")
    except Exception as e:
        raise errors.RadioError(f"Error exiting programming {e}")


def check_ident(data):
    if data in (b'P13GMRS', b'UV15999'):
        LOG.info(f"Radio is: {data}")
    else:
        err = (f"Ident returned unknown Radio: {data}")
        LOG.debug(err)
        raise errors.RadioError(err)


def do_sysinfo(serial):
    resp = raw_send(serial, b'PASSSTA', 3)
    if resp != b'\x50\x00\x00':
        raise errors.RadioError(f"Expected 0x500000 got {resp}")
    resp = raw_send(serial, b'SYSINFO', 1)
    if resp != b'\x06':
        raise errors.RadioError(f"ACK expected got {resp}")
    LOG.debug(f"SYSINFO: {resp}")


def do_readconfig(serial):
    # cmds start with 56, and expect 06 ack after recv ack
    for addr, _len in [(0x00000a0d, 13), (0x00100a0d, 13),
                       (0x00200a0d, 13), (0x0000000a, 11)]:
        cmd = struct.pack('>BL', 0x56, addr)
        resp = raw_send(serial, cmd, _len)
        if len(resp) != _len:
            raise errors.RadioError(f"Expected (_len) Bytes got {resp}")
        do_ack_ack(serial)
    return True


def do_prog2(serial, ident=b'P13GMRS'):
    cmd = struct.pack('>LB', 0xffffffff, 0x0c)
    serial.write(cmd)  # no resp expected
    resp = raw_send(serial, ident, 1)
    if resp != b'\x06':
        raise errors.RadioError(f"Error expected 06 ack got {resp}")
    resp = raw_send(serial, b'\x02', 8)
    if len(resp) != 8:
        raise errors.RadioError(f"Error expected len 8 got {resp}")
    do_ack_ack(serial)


NUM_TABLES = 4


def do_read_tlmap(serial, type_map=None, num_tables=NUM_TABLES):
    """ The radio has defined types of config data.
        Before reading or writing, we ask the radio where
        the data type is stored in memory. The radio
        moves memory locations possibly for durability.

        Critically, the location directory is NOT just one 16-slot
        table -- it is one 16-slot table PER "table index" (a 3rd
        addressing byte in the query, distinct from every other radio
        in this family we'd previously reverse-engineered). A given
        type can end up filed under table 0, table 1, table 2, etc,
        and which table varies session to session just like the
        location within a table does. Confirmed via a real USB capture
        of the vendor's own CPS software: it queries multiple tables
        (0..3) the same way, and on that capture 'settings' and the
        primary channel bank both ended up in table 1, not table 0.

        Build ephemeral map of type to (table, location). """
    if type_map is None:
        type_map = TYPE_MAP
    tl_map = {m[0]: None for m in type_map}
    for table in range(num_tables):
        if all(v is not None for v in tl_map.values()):
            break
        for i in range(0, 16):
            addr = (i << 4 | 0x0f)
            # ask the radio where each data type is in mem
            cmd = struct.pack('>4B', 0xff, addr, table, 0x01)
            resp = do_read_cmd(serial, cmd, 1)
            # mem location is for this prog session only
            r_int = struct.unpack('>B', resp)[0]
            if r_int in tl_map and tl_map[r_int] is None:
                tl_map[r_int] = (table, i << 4)
    return tl_map


def do_read_ranges(serial, table, loc, radio, status):
    # read each data/config type from the (table, loc)ation
    # obtained by tlmap query. each loc has 4 (pre)fix
    # block numbers. Each block is 64 bytes
    data = b''
    for i in range(16):
        for pre in range(0x0, 0xc1, 0x40):
            addr = struct.pack('>BBBB', pre, loc + i, table, 0x40)
            data += do_read_cmd(serial, addr, 0x40)
            status.cur += 0x40
            radio.status_fn(status)
    return data


def do_upload_block(serial, table, loc, offset, radio, status):
    # write each config type to (table, loc) retrieved from tlmap
    # 4 defined (pre)fixes/blocks per (loc)ation
    # pre = 0x00 , 0x40 , 0x80, 0xc0
    # full command example  [57][80][4d][table]0040
    # 57 = write , 80 prefix/block , 4d location to write
    _mem = radio._memobj.get_raw()
    for i in range(16):
        for pre in range(0x0, 0xc1, 0x40):
            x = offset + (i * 0x100) + pre
            block = _mem[x:x + 0x40]
            addr = struct.pack('>BBBBB', 0x57, pre, loc + i, table, 0x40)
            data = addr + block
            ack = raw_send(serial, data, 1)
            if ack != b'\x06':
                err = f"Bad ack on write expect 0x06 got {ack}"
                LOG.debug(err)
                raise errors.RadioError(err)
            status.cur += len(block)
            radio.status_fn(status)


def do_download(radio):
    data = b''
    try:
        status = chirp_common.Status()
        serial = radio.pipe
        serial.flush()
        ident = enter_prog(serial)
        check_ident(ident)
        do_sysinfo(serial)
        do_readconfig(serial)
        do_prog2(serial, ident)
        tl_map = do_read_tlmap(serial, getattr(radio, 'TYPE_MAP', None))
        status.max = len(tl_map) * 0x1000
        status.msg = "Downloading..."
        for t, loc in tl_map.items():
            if loc is None:
                raise errors.RadioError(f"TL Map failed {t} {loc}")
            table, loc_addr = loc
            data += do_read_ranges(serial, table, loc_addr, radio, status)
    except Exception as e:
        raise errors.RadioError(f"Error during download {e}")
    finally:
        exit_prog(serial)
    return memmap.MemoryMapBytes(data)


def do_upload(radio):
    try:
        write_map = getattr(radio, 'WRITE_MAP', None) or WRITE_MAP
        status = chirp_common.Status()
        status.max = len(write_map) * 0x1000
        serial = radio.pipe
        serial.flush()
        ident = enter_prog(serial)
        check_ident(ident)
        do_sysinfo(serial)
        do_readconfig(serial)
        do_prog2(serial, ident)
        tl_map = do_read_tlmap(serial, getattr(radio, 'TYPE_MAP', None))
        for wt, label, offset in write_map:
            status.msg = f"Uploading: {label}..."
            loc = tl_map[wt]
            if loc is None:
                raise errors.RadioError(f"TL Map failed {wt} {loc}")
            table, loc_addr = loc
            do_upload_block(serial, table, loc_addr, offset, radio, status)
    except errors.RadioError:
        raise
    except Exception as e:
        raise errors.RadioError(f"Error during upload {e}")
    finally:
        exit_prog(serial)


@directory.register
class RadioddityGM30(chirp_common.CloneModeRadio):
    """Radioddity GM-30"""
    VENDOR = "Radioddity"
    MODEL = "GM-30"
    BAUD_RATE = 57600
    POWER_LEVELS = [chirp_common.PowerLevel("Low",  watts=0.50),
                    chirp_common.PowerLevel("High", watts=3.00)]
    VALID_MODES = ["NFM", "FM"]
    _range = [(136000000, 174000000), (400000000, 470000000)]
    GMRS_RPTR = [462550000, 462575000, 462600000, 462625000, 462650000,
                 462675000, 462700000, 462725000]

    VALID_TONES = chirp_common.TONES
    VALID_DCS = [i for i in range(
        0, 778) if '9' not in str(i) and '8' not in str(i)]
    DCS_N = 0x80
    DCS_R = 0x40
    VALID_CHARSET = chirp_common.CHARSET_ASCII
    VALID_DTMF = [str(i) for i in range(0, 10)] + \
        ["A", "B", "C", "D", "*", "#"]
    ASCII_NUM = [str(i) for i in range(10)] + [' ']

    VALID_STEPS = [2.5, 5.0, 6.25, 10, 12.5, 20, 25, 50]

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_tuning_step = False
        rf.valid_tuning_steps = self.VALID_STEPS
        rf.has_name = True
        rf.valid_characters = self.VALID_CHARSET
        rf.valid_name_length = 6
        rf.has_offset = True
        rf.has_mode = True
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.can_odd_split = False
        rf.can_delete = True
        rf.valid_modes = self.VALID_MODES
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->DTCS",
            "DTCS->Tone",
            "->Tone",
            "Tone->Tone",
            "->DTCS",
            "DTCS->",
            "DTCS->DTCS"]
        rf.valid_power_levels = self.POWER_LEVELS
        rf.valid_skips = ["", "S"]
        rf.valid_bands = self._range
        rf.memory_bounds = (1, 250)
        return rf

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def sync_in(self):
        try:
            data = do_download(self)
        except errors.RadioError:
            raise
        except Exception as e:
            err = f'Error during download {e}'
            LOG.error(err)
            raise errors.RadioError(err)
        self._mmap = data
        self.process_mmap()

    def sync_out(self):
        try:
            do_upload(self)
        except Exception as e:
            err = f'Error during upload {e}'
            LOG.error(err)
            raise errors.RadioError(err)

    def val_or_def(self, memidx, _list):
        try:
            return _list[memidx]
        except IndexError:
            return _list[0]

    def idx_or_def(self, memitem, _list):
        try:
            return _list.index(memitem)
        except ValueError:
            return 0

    def get_str_name(self, _name):
        return ''.join(filter
                       (lambda x: x in self.VALID_CHARSET,
                        str(_name)))

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number - 1]) + \
               repr(self._memobj.memnames[number - 1])

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        _mem = self._memobj.memory[number-1]
        _name = self._memobj.memnames[number-1]["name"]
        str_name = self.get_str_name(_name)
        mem.name = str_name.rstrip()

        if _mem.rx_freq.get_raw() == b'\xff\xff\xff\xff':
            mem.empty = True
            return mem

        mem.freq = int(_mem.rx_freq) * 10

        if _mem.tx_freq.get_raw() == b'\xff\xff\xff\xff':
            mem.duplex = 'off'
        elif int(_mem.tx_freq) - int(_mem.rx_freq) > 0:
            # '+' duplex
            mem.duplex = '+'
            mem.offset = (int(_mem.tx_freq) - int(_mem.rx_freq)) * 10
        elif int(_mem.tx_freq) - int(_mem.rx_freq) < 0:
            # '-' duplex
            mem.duplex = '-'
            mem.offset = (int(_mem.rx_freq) - int(_mem.tx_freq)) * 10

        mem.mode = self.val_or_def(_mem.mode, self.VALID_MODES)
        mem.power = self.val_or_def(_mem.power, self.POWER_LEVELS)
        mem.skip = "" if _mem.scan else "S"

        if 1 <= mem.number <= 7 or 15 <= mem.number <= 22:
            mem.duplex = ""
            mem.immutable = ['duplex', 'offset', 'empty']
        elif 8 <= mem.number <= 14:
            mem.duplex = "off"
            mem.immutable += ['duplex', 'offset', 'empty', 'mode', 'power']
        elif 23 <= mem.number <= 54:
            mem.offset = 5 * 1000000
            mem.duplex = '+'
            mem.immutable = ['duplex', 'offset', 'empty']
        else:
            mem.offset = 0
            mem.duplex = "off"
            mem.immutable += ['offset', 'duplex']
        if 0 < mem.number <= 30:
            mem.immutable = mem.immutable + ['freq']

        txtone = self.get_tone(_mem.tx_tone)
        rxtone = self.get_tone(_mem.rx_tone)
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        mem.extra = RadioSettingGroup("Extra", "extra")

        rs = RadioSettingValueBoolean(_mem.busy_lock)
        rset = RadioSetting("busy_lock", "Busy Lock", rs)
        mem.extra.append(rset)

        rs = RadioSettingValueBoolean(_mem.freq_hop)
        rset = RadioSetting("freq_hop", "Freq. Hop", rs)
        mem.extra.append(rset)

        _current = _mem.signal if _mem.signal else 1
        rs = RadioSettingValueInteger(1, 15, current=_current)
        rset = RadioSetting("signal", "DTMF ID", rs)
        mem.extra.append(rset)

        options = ['Off', 'BOT', 'EOT', 'BOTH']
        rs = RadioSettingValueList(options, current_index=_mem.ptt_id)
        rset = RadioSetting("ptt_id", "PTT ID", rs)
        mem.extra.append(rset)

        return mem

    def validate_memory(self, mem):
        if 31 <= mem.number <= 54:
            if mem.freq not in self.GMRS_RPTR:
                return [chirp_common.ValidationError(
                    'Only GMRS repeater freq. permitted'
                    ' on channels 31 - 54')]
        return super().validate_memory(mem)

    def set_memory(self, mem):
        number = mem.number
        _mem = self._memobj.memory[number-1]
        _name = self._memobj.memnames[number-1]
        newname = [str(c) for c in mem.name]
        _name.name = "".join(newname).ljust(6, '\x00')

        if mem.empty:
            _mem.set_raw(b'\xff' * 13 + b'\x06\x11\x00')
            return

        _mem.rx_freq = mem.freq // 10

        if mem.duplex == "+":
            _mem.tx_freq = (mem.freq + mem.offset) // 10
        elif mem.duplex == "-":
            _mem.tx_freq = (mem.freq - mem.offset) // 10
        elif mem.duplex == "":
            _mem.tx_freq = _mem.rx_freq
        elif mem.duplex == "off":
            _mem.tx_freq.fill_raw(b'\xff')

        _mem.mode = self.idx_or_def(mem.mode, self.VALID_MODES)
        _mem.power = self.idx_or_def(mem.power, self.POWER_LEVELS)
        _mem.scan = False if mem.skip == "S" else True

        ((txmode, txval, txpol),
         (rxmode, rxval, rxpol)) = chirp_common.split_tone_encode(mem)
        self.set_tone(_mem.tx_tone, txmode, txval, txpol)
        self.set_tone(_mem.rx_tone, rxmode, rxval, rxpol)

        # extra settings
        for setting in mem.extra:
            setattr(_mem, setting.get_name(), setting.value)

    def get_tone(self, _memval):
        _memval[1].ignore_bits(self.DCS_N | self.DCS_R)
        dcsn = _memval[1].get_bits(self.DCS_N)
        dcsr = _memval[1].get_bits(self.DCS_R)
        rb = _memval[1].get_raw()
        if rb == b'\xff' or rb == b'\x00':
            return "", 0, None
        elif dcsn:
            pol = "R" if dcsr else "N"
            return "DTCS", int(_memval), pol
        else:
            return "Tone",  int(_memval) / 10, None

    def set_tone(self, _memval, mode, val, pol):
        # sets tones in _mem from ui edit
        _memval[1].ignore_bits(self.DCS_N | self.DCS_R)
        if mode == "":
            _memval.set_raw(b'\xff\xff')
        if mode == "Tone":
            _memval[1].clr_bits(self.DCS_N | self.DCS_R)
            _memval.set_value(val * 10)
        if mode == "DTCS":
            _memval[1].set_bits(self.DCS_N)
            if pol == 'R':
                _memval[1].set_bits(self.DCS_R)
            else:
                _memval[1].clr_bits(self.DCS_R)
            _memval.set_value(val)

    def get_settings(self):
        _settings = self._memobj.settings
        _dtmf = self._memobj.dtmf
        _dtmf_list = self._memobj.dtmf_list

        gsettings = RadioSettingGroup("gsettings", "General Settings")
        group = RadioSettings(gsettings)

        _options = ["Logo", "Message", "Voltage"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.bootscrmode)
        rset = RadioSetting("settings.bootscrmode", "Boot Screen Mode", rs)
        gsettings.append(rset)

        _current = "".join(chr(i) for i in _settings.bootscreen1
                           if chr(i) in self.VALID_CHARSET)
        rs = RadioSettingValueString(minlength=0, maxlength=10,
                                     current=_current,
                                     charset=self.VALID_CHARSET,
                                     mem_pad_char=' ')
        rset = RadioSetting("settings.bootscreen1",
                            "Boot Screen 1", rs)
        gsettings.append(rset)

        _current = "".join(chr(i) for i in _settings.bootscreen2
                           if chr(i) in self.VALID_CHARSET)
        rs = RadioSettingValueString(minlength=0, maxlength=10,
                                     current=_current,
                                     charset=self.VALID_CHARSET,
                                     mem_pad_char=' ')
        rset = RadioSetting("settings.bootscreen2",
                            "Boot Screen 2", rs)
        gsettings.append(rset)

        _options = [str(i) for i in range(0, 601, 15)]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.timeout)
        rset = RadioSetting("settings.timeout", "Timeout (s)", rs)
        gsettings.append(rset)

        rs = RadioSettingValueInteger(minval=0, maxval=9,
                                      current=_settings.squelch, step=1)
        rset = RadioSetting("settings.squelch", "Squelch Level", rs)
        gsettings.append(rset)

        rs = RadioSettingValueInteger(minval=0, maxval=9,
                                      current=_settings.vox_level, step=1)
        rset = RadioSetting("settings.vox_level", "Vox Level", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.voice_alert, mem_vals=(0, 1))
        rset = RadioSetting("settings.voice_alert", "Voice Alert", rs)
        gsettings.append(rset)

        _options = ["Freq. Mode", "Ch. Mode"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.work_mode)
        rset = RadioSetting("settings.work_mode", "Display Mode", rs)
        gsettings.append(rset)

        _options = ["None", "1:1", "1:2", "1:3", "1:4"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.batt_save)
        rset = RadioSetting("settings.batt_save", "Battery Save Mode", rs)
        gsettings.append(rset)

        _options = ["Bright", "1", "2", "3", "4", "5",
                    "6", "7", "8", "9", "10"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.backlight)
        rset = RadioSetting("settings.backlight", "Backlight", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.auto_key_lock, mem_vals=(0, 1))
        rset = RadioSetting("settings.auto_key_lock", "Auto Key Lock", rs)
        gsettings.append(rset)

        _options = ["Off", "DT-ST", "ANI-ST", "DT+ANI"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.side_tone)
        rset = RadioSetting("settings.side_tone", "DTMF Side Tone", rs)
        gsettings.append(rset)

        _options = ["Time", "Carrier", "Search"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.scan_type)
        rset = RadioSetting("settings.scan_type", "Scan Type", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.ctcss_revert, mem_vals=(0, 1))
        rset = RadioSetting("settings.ctcss_revert", "CTCSS Tail Revert", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.beep_tone, mem_vals=(0, 1))
        rset = RadioSetting("settings.beep_tone", "Beep Tone", rs)
        gsettings.append(rset)

        _options = ["On Site", "Send Sound", "Send Code"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.alarm_mode)
        rset = RadioSetting("settings.alarm_mode", "Alarm Mode", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.fm_radio, mem_vals=(0, 1))
        rset = RadioSetting("settings.fm_radio", "FM Radio", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.roger, mem_vals=(0, 1))
        rset = RadioSetting("settings.roger", "Roger Beep", rs)
        gsettings.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.standby, mem_vals=(0, 1))
        rset = RadioSetting("settings.standby", "Dual Standby", rs)
        gsettings.append(rset)

        _options = [str(i) for i in range(0, 1001, 100)]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.tail_revert)
        rset = RadioSetting("settings.tail_revert",
                            "Repeater Tail Revert (ms)", rs)
        gsettings.append(rset)

        # same options as tail_rvt
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.tail_delay)
        rset = RadioSetting("settings.tail_delay",
                            "Repeater Tail Delay (ms)", rs)
        gsettings.append(rset)

        _options = ["1000", "1450", "1750", "2100"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.tbst)
        rset = RadioSetting("settings.tbst", "Tone Burst", rs)
        gsettings.append(rset)

        _options = ["Name + Number", "Freq. + Number"]
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.a_ch_disp)
        rset = RadioSetting("settings.a_ch_disp", "A Channel Display Type", rs)
        gsettings.append(rset)

        # same options as a_chan_disp
        rs = RadioSettingValueList(_options,
                                   current_index=_settings.b_ch_disp)
        rset = RadioSetting("settings.b_ch_disp", "B Channel Display Type", rs)
        gsettings.append(rset)

        # DTMF Menu
        dtmf = RadioSettingGroup("dtmf", "DTMF")
        group.append(dtmf)

        def _dtmf_decode(setting, pad_len):
            _map = list(range(10)) + ['A', 'B', 'C', 'D', '*', '#']
            s = ""
            for i in setting:
                _i = int(i)
                if _i < len(_map):
                    s += str(_map[_i])
            return s.ljust(pad_len)

        _current = _dtmf_decode(_dtmf.radio_id, 5)
        rs = RadioSettingValueString(
            minlength=5, maxlength=5, current=_current)
        rset = RadioSetting("dtmf.radio_id", "Radio ID", rs)
        dtmf.append(rset)

        rs = RadioSettingValueBoolean(
            current=_dtmf.press_send, mem_vals=(0, 1))
        rset = RadioSetting("dtmf.press_send", "PTT Press Send", rs)
        dtmf.append(rset)

        rs = RadioSettingValueBoolean(
            current=_dtmf.release_send, mem_vals=(0, 1))
        rset = RadioSetting("dtmf.release_send", "PTT Release Send", rs)
        dtmf.append(rset)

        _options = [str(i) for i in range(100, 1010, 50)]
        rs = RadioSettingValueList(_options,
                                   current_index=_dtmf.delay_time)
        rset = RadioSetting("dtmf.delay_time", "Delay Time (ms)", rs)
        dtmf.append(rset)

        _options = [str(i) for i in range(80, 2010, 10)]
        rs = RadioSettingValueList(_options,
                                   current_index=_dtmf.digit_dur)
        rset = RadioSetting("dtmf.digit_dur", "Digit Duration (ms)", rs)
        dtmf.append(rset)

        # uses same options as digit_dur
        rs = RadioSettingValueList(_options,
                                   current_index=_dtmf.inter_dur)
        rset = RadioSetting(
            "dtmf.inter_dur", "Digit Interval Duration (ms)", rs)
        dtmf.append(rset)

        # DTMF Entries List
        dtmflist = RadioSettingGroup("dtmflist", "DTMF List")
        group.append(dtmflist)
        for i in range(0, 15):  # Entries # 1-15
            rs = RadioSettingValueString(
                minlength=0, maxlength=5,
                current=_dtmf_decode(_dtmf_list[i].entry, 5),
                autopad=False,
                charset=self.VALID_DTMF + [' '])
            rset = RadioSetting(f"dtmf_list[{i}].entry", f"Entry {i+1}", rs)
            dtmflist.append(rset)

        # Password Menu
        pro = RadioSettingGroup("protect", "Protect")
        group.append(pro)

        def _ascii_num_filter(setting):
            s = ""
            for i in setting:
                if chr(i) in self.ASCII_NUM:
                    s += str(chr(i))
            return s

        rs = RadioSettingValueBoolean(
            current=_settings.passw_w_ena, mem_vals=(0, 1))
        rs.set_mutable(False)
        rset = RadioSetting("settings.passw_w_ena", "Write Protect", rs)
        pro.append(rset)

        _current = _ascii_num_filter(_settings.passw_w_val)
        rs = RadioSettingValueString(
            minlength=0, maxlength=8, current=_current,
            charset=self.ASCII_NUM)
        rs.set_mutable(False)
        rset = RadioSetting("settings.passw_w_val", "Write Password", rs)
        pro.append(rset)

        rs = RadioSettingValueBoolean(
            current=_settings.passw_r_ena, mem_vals=(0, 1))
        rs.set_mutable(False)
        rset = RadioSetting("settings.passw_r_ena", "Read Protect", rs)
        pro.append(rset)

        _current = _ascii_num_filter(_settings.passw_r_val)
        rs = RadioSettingValueString(
            minlength=0, maxlength=8, current=_current,
            charset=self.ASCII_NUM)
        rs.set_mutable(False)
        rset = RadioSetting("settings.passw_r_val", "Read Password", rs)
        pro.append(rset)

        return group

    def _ff_pad__mem(self, obj, setting, element, charset, allow_space=False):
        """ set_settings helper for 0xff padded elements
            optional remove space chars
        """
        _charset = [c for c in list(charset)]
        if not allow_space:
            _charset = [c for c in list(charset) if c != ' ']
        _val = [0xff] * len(obj[setting])
        _i = 0  # offset for invalid chars
        for i in range(len(obj[setting])):
            if element.value[i] in _charset:
                _val[i - _i] = ord(element.value[i])
            else:
                _i += 1
        setattr(obj, setting, _val)

    def _dtmf_set__mem(self, obj, setting, element):
        _dtmf_map = [str(i) for i in range(10)]
        _dtmf_map += ['A', 'B', 'C', 'D', '*', '#']
        _val = [0xff] * len(obj[setting])
        _os = 0  # offset _mem setting idx for pad chars
        for i in range(len(element.value)):
            if element.value[i] in _dtmf_map:
                _val[i - _os] = _dtmf_map.index(element.value[i])
            else:
                _os += 1
        setattr(obj, setting, _val)

    def set_settings(self, settings):
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            else:
                try:
                    if "." in element.get_name():
                        toks = element.get_name().split(".")
                        obj = self._memobj
                        for tok in toks[:-1]:
                            if '[' in tok:
                                t, i = tok.split("[")
                                i = int(i[:-1])
                                obj = getattr(obj, t)[i]
                            else:
                                obj = getattr(obj, tok)
                        setting = toks[-1]
                    else:
                        obj = self._memobj.settings
                        setting = element.get_name()

                    if element.has_apply_callback():
                        LOG.debug("applying callback")
                        element.run_apply_callback()

                    if element.value.get_mutable():
                        if setting in ['passw_w_val', 'passw_r_val']:
                            self._ff_pad__mem(
                                obj, setting, element, self.ASCII_NUM)
                        elif setting in ['bootscreen1', 'bootscreen2']:
                            self._ff_pad__mem(
                                obj, setting, element, self.VALID_CHARSET,
                                allow_space=True)
                        elif setting == 'entry':
                            self._dtmf_set__mem(
                                obj, setting, element)
                        else:
                            setattr(obj, setting, element.value)
                except Exception:
                    LOG.debug(element.get_name())
                    raise


@directory.register
class BaofengGM15Pro(RadioddityGM30):
    """Baofeng GM-15Pro"""
    VENDOR = "Baofeng"
    MODEL = "GM-15Pro"
    _range = [(136000000, 174000000), (400000000, 512000000)]


@directory.register
class RadioddityMU5(RadioddityGM30):
    """Radioddity MU-5 (MURS)

    Identical to Radioddity GM-30 except for the following:

    MURS channels 1-20 are fixed to the 5 MURS frequencies in a repeating
    pattern. Frequency in EEPROM is ignored for 1-20. Channels 21-250 are
    user-programmable RX-only channels. Changing power level is not allowed
    on 1-20. It is possible to set power level on 21-250, but it is not used
    (radio forces RX-only).
    """
    VENDOR = "Radioddity"
    MODEL = "MU-5"

    # MU-5 does not actually support variable power levels
    # MURS channels are fixed to "Low" power which is presumably 2W
    POWER_LEVELS = [chirp_common.PowerLevel("Low",  watts=2.00),
                    chirp_common.PowerLevel("High", watts=2.00)]

    MURS_FREQS = [151820000, 151880000, 151940000, 154570000, 154600000]

    def _get_murs_freq(self, channel):
        return self.MURS_FREQS[(channel - 1) % 5]

    def _is_murs_narrowband(self, channel):
        return ((channel - 1) % 5) < 3

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        _mem = self._memobj.memory[number-1]
        _name = self._memobj.memnames[number-1]["name"]
        str_name = self.get_str_name(_name)
        mem.name = str_name.rstrip()

        # 1-20 always exist and may have 0xFFFFFFFF for freq
        # other channels with 0xFFFFFFFF for freq are empty
        if _mem.rx_freq.get_raw() == b'\xff\xff\xff\xff':
            if not (1 <= number <= 20):
                mem.empty = True
                return mem

        mem.freq = int(_mem.rx_freq) * 10

        if _mem.tx_freq.get_raw() == b'\xff\xff\xff\xff':
            mem.duplex = 'off'
        elif int(_mem.tx_freq) - int(_mem.rx_freq) > 0:
            mem.duplex = '+'
            mem.offset = (int(_mem.tx_freq) - int(_mem.rx_freq)) * 10
        elif int(_mem.tx_freq) - int(_mem.rx_freq) < 0:
            mem.duplex = '-'
            mem.offset = (int(_mem.rx_freq) - int(_mem.tx_freq)) * 10

        mem.mode = self.val_or_def(_mem.mode, self.VALID_MODES)
        mem.power = self.val_or_def(_mem.power, self.POWER_LEVELS)
        mem.skip = "" if _mem.scan else "S"

        # MURS channels 1-20: fixed freq, simplex, fixed power
        if 1 <= mem.number <= 20:
            mem.freq = self._get_murs_freq(mem.number)
            mem.duplex = ""
            mem.offset = 0
            mem.immutable = ['freq', 'duplex', 'offset', 'power', 'empty']
            # Narrowband channels have fixed mode
            if self._is_murs_narrowband(mem.number):
                mem.mode = "NFM"
                mem.immutable = mem.immutable + ['mode']
        # Channels 21-250: RX-only user channels
        elif mem.number > 20:
            mem.duplex = "off"
            mem.immutable = ['duplex', 'offset']

        txtone = self.get_tone(_mem.tx_tone)
        rxtone = self.get_tone(_mem.rx_tone)
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        mem.extra = RadioSettingGroup("Extra", "extra")

        rs = RadioSettingValueBoolean(_mem.busy_lock)
        rset = RadioSetting("busy_lock", "Busy Lock", rs)
        mem.extra.append(rset)

        rs = RadioSettingValueBoolean(_mem.freq_hop)
        rset = RadioSetting("freq_hop", "Freq. Hop", rs)
        mem.extra.append(rset)

        _current = _mem.signal if _mem.signal else 1
        rs = RadioSettingValueInteger(1, 15, current=_current)
        rset = RadioSetting("signal", "DTMF ID", rs)
        mem.extra.append(rset)

        options = ['Off', 'BOT', 'EOT', 'BOTH']
        rs = RadioSettingValueList(options, current_index=_mem.ptt_id)
        rset = RadioSetting("ptt_id", "PTT ID", rs)
        mem.extra.append(rset)

        return mem

    def validate_memory(self, mem):
        if 1 <= mem.number <= 20:
            expected_freq = self._get_murs_freq(mem.number)
            if mem.freq != expected_freq:
                return [chirp_common.ValidationError(
                    f'MURS Channel {mem.number} must be '
                    f'{expected_freq / 1000000:.3f} MHz')]
            if self._is_murs_narrowband(mem.number) and mem.mode != "NFM":
                return [chirp_common.ValidationError(
                    f'MURS Channel {mem.number} must be narrowband (NFM)')]
        return chirp_common.CloneModeRadio.validate_memory(self, mem)

    def set_memory(self, mem):
        # For MURS channels 1-20, write 0xFFFFFFFF for rx/tx freq
        # The radio firmware hardcodes the actual MURS frequency
        # This is what the official CPS does
        if 1 <= mem.number <= 20:
            number = mem.number
            _mem = self._memobj.memory[number-1]
            _name = self._memobj.memnames[number-1]
            newname = [str(c) for c in mem.name]
            _name.name = "".join(newname).ljust(6, '\x00')

            if mem.empty:
                _mem.set_raw(b'\xff' * 13 + b'\x06\x11\x00')
                return

            _mem.rx_freq.fill_raw(b'\xff')
            _mem.tx_freq.fill_raw(b'\xff')

            _mem.mode = self.idx_or_def(mem.mode, self.VALID_MODES)
            _mem.power = self.idx_or_def(mem.power, self.POWER_LEVELS)
            _mem.scan = False if mem.skip == "S" else True

            ((txmode, txval, txpol),
             (rxmode, rxval, rxpol)) = chirp_common.split_tone_encode(mem)
            self.set_tone(_mem.tx_tone, txmode, txval, txpol)
            self.set_tone(_mem.rx_tone, rxmode, rxval, rxpol)

            for setting in mem.extra:
                setattr(_mem, setting.get_name(), setting.value)
        else:
            super().set_memory(mem)


# --- Experimental Baofeng UV-15Pro support ---------------------------------
#
# Reverse-engineered from a live radio (not from vendor documentation),
# cross-checked against multiple real USB captures of the vendor's own
# Windows CPS software. See README.md in this driver's repository for
# the full protocol write-up.
#
# The UV-15Pro speaks the same handshake/protocol as the GM-30 family but
# adds a second addressing byte ("table", 0-3) to every read/write command
# that the GM-30/GM-15Pro/MU-5 never needed, and spreads its 999 channels
# across four fixed-size banks (memory/memory17/memory18/special) instead
# of one.
#
# Download and upload (settings, DTMF, all four channel banks, VFO A/B)
# have all been verified round-trip against a real unit and cross-checked
# against the vendor CPS, but this is still a new, from-scratch driver for
# an otherwise-undocumented radio -- treat it as experimental.
MEM_FORMAT_UV15PRO = """
struct{
    u8     unknown02[256];
} unknown02[16];
struct{
    u8      bootscrmode;
    u8      bsmodepad[15];
    u8      bootscreen1[10];
    u8      bs1pad[6];
    u8      bootscreen2[10];
    u8      bs2pad[6];
    u8      unused[16];
    u8      timeout;
    u8      squelch;
    u8      vox_level;
    u8      batt_save:4,
            unk_bits:2,
            work_mode:1,
            voice_alert:1;
    u8      backlight;
    u8      beep_tone:1,
            auto_key_lock:1,
            unk_bit_2:1,
            ctcss_revert:1,
            scan_type:2,
            side_tone:2;
    u8      unk_bit_3:1,
            standby:1,
            roger:1,
            alarm_mode:2,
            alarm_sound:1,
            fm_radio:1,
            unk_bit_4:1;
    u8      tail_revert;
    u8      tail_delay;
    u8      tbst;
    u8      unk_sett[6];
    u8      unk_bits_5:6,
            b_ch_disp:1,
            a_ch_disp:1;
    u8      unk_sett_2[24];
    u8      passw_w_ena;
    u8      passw_r_ena;
    u8      passw_w_val[8];
    u8      passw_r_val[8];
    u8      unk_sett_3[5];
} settings;
struct{
    u8     unusedsettings[32];
} unusedsettings[124];
struct{
    u8     entry[5];
} dtmf_list[15];
struct{
    u8      dtmf_l_pad[5];
    u8      radio_id[5];
    u8      unk_dtmf[11];
    u8      unk_dtmf_bits:6,
            press_send:1,
            release_send:1;
    u8      delay_time;
    u8      digit_dur;
    u8      inter_dur;
    u8      unk_dtmf_2[28];
} dtmf;
struct{
    u8     unkdtmf[128];
} unuseddtmf[31];
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} memory[256];
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} memory17[256];
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} memory18[256];
struct {
    lbcd      rx_freq[4];
    lbcd      tx_freq[4];
    u8        mem_changed;
    lbcd      rx_tone[2];
    lbcd      tx_tone[2];
    u8      unk_mem2:1,
            busy_lock:1,
            ptt_id:2,
            unk_mem3:1,
            mode:1,
            power:1,
            unk_mem4:1;
            u8 signal:4,
            unk_mem5:2,
            freq_hop:1,
            scan:1;
            u8 step;
} special[256];
struct{
    char      name[6];
    u8      pad[5];
} memnames[372];
struct{
    u8     pad[4];
} names_pad;
struct{
    u8     raw[4096];
} unknown25;
struct{
    char      name[6];
    u8      pad[5];
} special_names[372];
struct{
    u8     pad[4];
} names26_pad;
"""

# Every one of this radio's channel banks is a fixed-size 256-record
# block whose VERY LAST record (255) is a self-tag end-marker -- 15
# bytes of 0xFF followed by the bank's own type byte (confirmed
# identically across all four banks found so far: 0x16/0x17/0x18/0x19
# each show this exact pattern at record 255). So every bank's real
# usable capacity is records 0-254 (255 records), not 0-255.
#
# Bank 1 (type 0x16) is the one exception with a reserved HEADER too:
# records 0-2 hold non-channel data (some kind of block metadata, not
# yet understood), so its usable channels start at record 3, not 0.
# Confirmed via a real USB capture of the vendor's CPS software:
# record 3 == channel 1 (454.22500 MHz) through record 19 == channel
# 17 (468.95000 MHz).
PRIMARY_CHANNEL_RECORD_OFFSET = 2   # channel_number = record_index - 2
PRIMARY_CHANNEL_FIRST_RECORD = 3
PRIMARY_CHANNEL_LAST_RECORD = 254   # record 255 is the type self-tag

# Bank 2 (type 0x17) has NO reserved header (confirmed: records 0-2
# were plain 0xFF before we ever touched them, unlike bank 1's odd
# header content) -- so its full records 0-254 are real channels,
# starting right where bank 1 left off. Originally confirmed via a
# real USB capture of the vendor's CPS writing channel 300 (landed at
# record 47, 47+253==300); the record 0-2 boundary was corrected after
# directly testing channels 253/254 (they write and read back fine).
BANK2_CHANNEL_OFFSET = 253   # channel_number = record_index + 253
BANK2_FIRST_RECORD = 0
BANK2_LAST_RECORD = 254   # record 255 is the type self-tag
BANK2_CHANNEL_MIN = 253
BANK2_CHANNEL_MAX = 507

# Bank 3 (type 0x18) follows the identical clean (no-header) pattern,
# continuing right where bank 2 left off. Confirmed: wrote a test
# channel predicted to land at channel 600 (record 91, 91+509==600),
# verified via the vendor's own CPS software reading the radio. (Side
# note: the radio's own front-panel keypad won't navigate to channel
# 600 specifically, even though 599 works and the CPS shows 600 is
# valid -- a firmware UI quirk unrelated to the data itself, and
# doesn't affect this driver since we talk to the radio over the
# serial protocol directly, not through its keypad UI.) Capped one
# channel short of the record-255 self-tag boundary (762 instead of
# 763) to avoid colliding with the special bank's confirmed start.
BANK3_CHANNEL_OFFSET = 509   # channel_number = record_index + 509
BANK3_FIRST_RECORD = 0
# record 254 would be channel 763, colliding with the special bank
# below (see BANK3_GAP_RECORD/BANK3_GAP_CHANNEL); 255 is the type
# self-tag
BANK3_LAST_RECORD = 253
BANK3_CHANNEL_MIN = 509
BANK3_CHANNEL_MAX = 762

# memory18 record 254 is real, ordinary storage -- NOT reserved/special
# data -- but the record_index+509 formula used for the rest of bank 3
# would number it channel 763, colliding with the special bank's own
# (separately confirmed) channel 763. Rather than shift the whole
# bank's offset (which is anchored to a real USB-captured vendor CPS
# write of channel 600, and shifting it would misalign every other
# channel in the bank against the vendor's own numbering), this one
# record is exposed under the single channel number that's otherwise
# unclaimed: 508. Confirmed safe to claim: read directly off a real
# unit, it held the exact same 16-byte blank-channel pattern
# (13x0xFF + 0x06 0x11 0x00) as every other ordinary empty channel
# elsewhere in this same bank -- not the kind of distinctive non-blank
# byte pattern bank 1's real reserved header records show (e.g.
# memory[0], memory[1]/VFO A).
BANK3_GAP_RECORD = 254
BANK3_GAP_CHANNEL = 508

# The 'special' block (type 0x19) and 'special_names' block (type
# 0x26) -- despite the name, there's nothing architecturally special
# about this bank; it's the same 256-record layout as banks 2/3 (no
# header, record 255 is the type self-tag), and the confirmed contents
# (channels 985-999: SOS, 7-7, REMERx, UREx) were simply put there
# manually for convenience, not reserved by firmware. Its true start
# is channel 763 (continuing the bank1->2->3->4 sequence), confirmed
# by decoding real content: record 222 == channel 985 (URE-0) through
# record 236 == channel 999 (SOS). Read/write both verified round-trip
# against a real unit and cross-checked against the vendor CPS.
# The name array (type 0x24, shared/reused here as 'special_names')
# uses a different stride (11 vs 16 bytes) so its own index-to-channel
# offset differs from the frequency side.
SPECIAL_FREQ_CHANNEL_OFFSET = 763   # channel_number = local_index + 763
SPECIAL_NAME_CHANNEL_OFFSET = 745   # channel_number = local_index + 745

# special_names is addressable down to local index 0 (channel 745),
# 18 channels before special's own frequency data starts at 763 --
# channels 745-762 already have real frequency storage of their own,
# in bank 3 (memory18, since BANK3_CHANNEL_MIN=509 <=745..762<=
# BANK3_CHANNEL_MAX=762). Read directly off a real unit: that low
# slice of special_names was clean, untouched blank storage (not
# corrupted leftover data from something else), and a write/read-back
# round trip to it left bank 3's own frequency data for those channel
# numbers, and special_names' own already-used higher entries,
# untouched. So channels 745-762 can have BOTH a real frequency (from
# bank 3) and a real name (from this extra slice of special_names) --
# they're independently-addressed storage, no conflict.
NAME_ONLY_CHANNEL_MIN = 745
NAME_ONLY_CHANNEL_MAX = BANK3_CHANNEL_MAX  # 762
SPECIAL_CHANNEL_MIN = 763
SPECIAL_CHANNEL_MAX = 999


@directory.register
class BaofengUV15Pro(RadioddityGM30):
    """Baofeng UV-15Pro (experimental)"""
    VENDOR = "Baofeng"
    MODEL = "UV-15Pro"
    _range = [(136000000, 174000000), (400000000, 512000000)]

    TYPE_MAP = [
        (0x02, "unknown02"),
        (0x04, "settings"),
        (0x06, "dtmf"),
        (0x16, "memory"),
        (0x17, "memory17"),
        (0x18, "memory18"),
        (0x19, "special"),
        (0x24, "chan_names"),
        (0x25, "unknown25"),
        (0x26, "special_names"),
    ]
    WRITE_MAP = [
        (0x04, "settings", 0x1000),
        (0x06, "dtmf", 0x2000),
        (0x16, "memory", 0x3000),
        (0x17, "memory17", 0x4000),
        (0x18, "memory18", 0x5000),
        (0x19, "special", 0x6000),
        (0x24, "chan_names", 0x7000),
        (0x26, "special_names", 0x9000),
    ]

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT_UV15PRO, self._mmap)

    def _closest_power_idx(self, power):
        """Map an arbitrary source power level to the closest of this
        radio's actual levels, by dBm. idx_or_def's exact-object match
        silently falls back to index 0 (Low) for any source that
        doesn't happen to define an identical PowerLevel object -- e.g.
        an imported CSV's "5.0W" never equals our own "High (3.0W)",
        so every imported channel was landing on Low power (0.4W)
        instead of High. Match by closest dBm instead (PowerLevel only
        reliably compares via float(), which is dBm)."""
        if power is None:
            return len(self.POWER_LEVELS) - 1  # default to highest
        target = float(power)
        diffs = [abs(float(pl) - target) for pl in self.POWER_LEVELS]
        return diffs.index(min(diffs))

    def get_settings(self):
        group = super().get_settings()
        gsettings = group["gsettings"]

        # The base GM-30 driver labels this bit "work_mode" (Freq.
        # Mode / Ch. Mode), inherited from a different model where
        # that's presumably correct. On the UV-15Pro it's actually
        # Language: confirmed via two codeplug exports differing in
        # exactly this one bit, with only the CPS's Language dropdown
        # touched (Work Mode itself was verified unchanged). Keep the
        # underlying field name as-is (the base class's get_settings
        # accesses it directly by that name before we get a chance to
        # intervene) -- just replace the mislabeled control with a
        # correctly-named one over the same bit/dotted-path, so
        # set_settings still routes writes to the right place.
        old = gsettings["settings.work_mode"]
        del gsettings[old]
        _options = ["Chinese", "English"]
        rs = RadioSettingValueList(
            _options, current_index=self._memobj.settings.work_mode)
        rset = RadioSetting("settings.work_mode", "Language", rs)
        gsettings.append(rset)

        # alarm_sound is a real, named bit in the settings struct (see
        # MEM_FORMAT_UV15PRO) that the base GM-30 driver's get_settings
        # never exposed a control for -- confirmed present and checked
        # in the vendor's own CPS ("Alarm Sound").
        rs = RadioSettingValueBoolean(
            current=self._memobj.settings.alarm_sound, mem_vals=(0, 1))
        rset = RadioSetting("settings.alarm_sound", "Alarm Sound", rs)
        gsettings.append(rset)

        # VFO A and VFO B: confirmed these are memory[1] and memory[2]
        # (ordinary 16-byte channel records, using the exact same
        # struct as every stored channel) -- what we'd previously
        # treated as an opaque "reserved header" before bank 1's real
        # channels start at record 3. Verified against the vendor's
        # CPS: its displayed VFO B (447.27500/440.12500MHz, tone
        # 107.2/107.2) matches memory[2] byte-for-byte. This matches
        # the original GM-30 driver's own model, which calls this same
        # 48-byte region 'tuning' (16 bytes, still unidentified) +
        # 'vfomemory[2]' (32 bytes, VFO A/B) immediately before its
        # channel memory array.
        vfo_group = RadioSettingGroup("vfo", "VFO A/B")
        for idx, label in ((1, "VFO A"), (2, "VFO B")):
            _mem = self._memobj.memory[idx]
            name = f"memory[{idx}]"

            rs = RadioSettingValueInteger(
                0, 99999999, current=int(_mem.rx_freq))
            rset = RadioSetting(f"{name}.rx_freq",
                                f"{label} RX Freq (x10Hz)", rs)
            vfo_group.append(rset)

            rs = RadioSettingValueInteger(
                0, 99999999, current=int(_mem.tx_freq))
            rset = RadioSetting(f"{name}.tx_freq",
                                f"{label} TX Freq (x10Hz)", rs)
            vfo_group.append(rset)

            rs = RadioSettingValueList(
                self.VALID_MODES, current_index=_mem.mode)
            rset = RadioSetting(f"{name}.mode", f"{label} Mode", rs)
            vfo_group.append(rset)

            _plabels = [str(p) for p in self.POWER_LEVELS]
            rs = RadioSettingValueList(
                _plabels, current_index=_mem.power)
            rset = RadioSetting(f"{name}.power", f"{label} Power", rs)
            vfo_group.append(rset)
        group.append(vfo_group)

        return group

    def get_features(self):
        rf = super().get_features()
        rf.memory_bounds = (1, SPECIAL_CHANNEL_MAX)
        return rf

    def validate_memory(self, mem):
        # No fixed GMRS channel plan on this model (unlike GM-30/GM-15Pro)
        return chirp_common.CloneModeRadio.validate_memory(self, mem)

    def _mem_objs(self, number):
        """Returns (_mem, _name, editable) for a channel number.

        Names for channels 1-372 live in 'memnames' (type 0x24, indexed
        directly by channel-1 -- confirmed: channel 1's name 'W-CH1'
        matches memnames[0], and channel 300's 'TEST300' matches
        memnames[299], showing the array holds more than the 256 we
        originally assumed -- 4096 bytes / 11 bytes each = 372).
        Frequencies for channels 1-252 live in 'memory' (type 0x16) --
        confirmed: channel 1 == 454.22500MHz == memory record 3,
        channel 17 == 468.95000MHz == record 19 (records 0-2 are a
        reserved header, not channels). Frequencies for channels
        253-507 live in 'memory17' (type 0x17), a second bank with NO
        header (unlike bank 1) -- confirmed via a real USB capture of
        the vendor's CPS writing channel 300 (landed at memory17 record
        47, 47+253=300), then double-checked by directly testing
        channels 253/254 against the radio (an earlier assumption that
        bank 2 shared bank 1's 3-record header, which would have put
        its first channel at 256, was wrong and has been corrected).
        Frequencies for channels 509-762 live in 'memory18' (type
        0x18), a third bank following the same no-header pattern, plus
        one more real record (memory18[254]) that the record_index+509
        formula can't reach without colliding with the special bank's
        channel 763 -- exposed instead as channel 508 (see
        BANK3_GAP_RECORD/BANK3_GAP_CHANNEL above). Channels in the
        confirmed special range (763-999) live in 'special'/
        'special_names' (type 0x19/0x26); read and write both verified
        round-trip. Anything outside these confirmed ranges isn't
        backed by known storage.

        Names are a separate concern from frequency storage. Besides
        'memnames' (channels 1-372), 'special_names' is itself
        addressable a bit further down than 'special' (the frequency
        side) is -- down to channel 745, 18 channels before 'special'
        starts at 763 (see NAME_ONLY_CHANNEL_MIN/MAX above). Those 18
        channels (745-762) already have real frequency storage of
        their own via bank 3 ('memory18'), so they end up with BOTH a
        name (from this extra slice of 'special_names') and a
        frequency (from bank 3) -- independently-addressed storage,
        no conflict. Verified via a write/read-back round trip against
        a real unit that didn't disturb bank 3's own data there or
        special_names' already-used higher entries.
        """
        if 1 <= number <= 372:
            _name = self._memobj.memnames[number - 1]
        elif NAME_ONLY_CHANNEL_MIN <= number <= SPECIAL_CHANNEL_MAX:
            ni = number - SPECIAL_NAME_CHANNEL_OFFSET
            _name = self._memobj.special_names[ni] if 0 <= ni < 372 else None
        else:
            _name = None
        if number == BANK3_GAP_CHANNEL:
            _mem = self._memobj.memory18[BANK3_GAP_RECORD]
            return (_mem, _name, True)
        ri = number + PRIMARY_CHANNEL_RECORD_OFFSET
        if (PRIMARY_CHANNEL_FIRST_RECORD <= ri <= PRIMARY_CHANNEL_LAST_RECORD):
            _mem = self._memobj.memory[ri]
            return (_mem, _name, _mem is not None)
        if BANK2_CHANNEL_MIN <= number <= BANK2_CHANNEL_MAX:
            ri = number - BANK2_CHANNEL_OFFSET
            _mem = (self._memobj.memory17[ri]
                    if BANK2_FIRST_RECORD <= ri <= BANK2_LAST_RECORD
                    else None)
            return (_mem, _name, _mem is not None)
        if BANK3_CHANNEL_MIN <= number <= BANK3_CHANNEL_MAX:
            ri = number - BANK3_CHANNEL_OFFSET
            _mem = (self._memobj.memory18[ri]
                    if BANK3_FIRST_RECORD <= ri <= BANK3_LAST_RECORD
                    else None)
            return (_mem, _name, _mem is not None)
        if SPECIAL_CHANNEL_MIN <= number <= SPECIAL_CHANNEL_MAX:
            fi = number - SPECIAL_FREQ_CHANNEL_OFFSET
            # _name already resolved above (NAME_ONLY_CHANNEL_MIN..MAX
            # covers this whole range too)
            _mem = self._memobj.special[fi] if 0 <= fi < 255 else None
            return (_mem, _name, _mem is not None)
        return (None, None, False)

    def get_raw_memory(self, number):
        _mem, _name, _ = self._mem_objs(number)
        return repr(_mem) + repr(_name)

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        _mem, _name, editable = self._mem_objs(number)

        if _mem is None:
            mem.empty = True
            mem.immutable = ['empty', 'freq', 'name', 'duplex', 'offset',
                             'mode', 'power', 'tmode']
            return mem

        if _name is not None:
            str_name = self.get_str_name(_name["name"])
            mem.name = str_name.rstrip()
        else:
            mem.immutable = ['name']

        if _mem.rx_freq.get_raw() == b'\xff\xff\xff\xff':
            mem.empty = True
            if not editable:
                mem.immutable = ['empty', 'freq', 'name', 'duplex',
                                 'offset', 'mode', 'power', 'tmode']
            return mem

        mem.freq = int(_mem.rx_freq) * 10

        if _mem.tx_freq.get_raw() == b'\xff\xff\xff\xff':
            mem.duplex = 'off'
        elif int(_mem.tx_freq) - int(_mem.rx_freq) > 0:
            mem.duplex = '+'
            mem.offset = (int(_mem.tx_freq) - int(_mem.rx_freq)) * 10
        elif int(_mem.tx_freq) - int(_mem.rx_freq) < 0:
            mem.duplex = '-'
            mem.offset = (int(_mem.rx_freq) - int(_mem.tx_freq)) * 10
        else:
            mem.duplex = ''

        mem.mode = self.val_or_def(_mem.mode, self.VALID_MODES)
        mem.power = self.val_or_def(_mem.power, self.POWER_LEVELS)
        mem.skip = "" if _mem.scan else "S"

        txtone = self.get_tone(_mem.tx_tone)
        rxtone = self.get_tone(_mem.rx_tone)
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        mem.extra = RadioSettingGroup("Extra", "extra")

        rs = RadioSettingValueBoolean(_mem.busy_lock)
        rset = RadioSetting("busy_lock", "Busy Lock", rs)
        mem.extra.append(rset)

        rs = RadioSettingValueBoolean(_mem.freq_hop)
        rset = RadioSetting("freq_hop", "Freq. Hop", rs)
        mem.extra.append(rset)

        _current = _mem.signal if _mem.signal else 1
        rs = RadioSettingValueInteger(1, 15, current=_current)
        rset = RadioSetting("signal", "DTMF ID", rs)
        mem.extra.append(rset)

        options = ['Off', 'BOT', 'EOT', 'BOTH']
        rs = RadioSettingValueList(options, current_index=_mem.ptt_id)
        rset = RadioSetting("ptt_id", "PTT ID", rs)
        mem.extra.append(rset)

        if not editable:
            mem.immutable = ['empty', 'freq', 'name', 'duplex', 'offset',
                             'mode', 'power', 'tmode']

        return mem

    def set_memory(self, mem):
        number = mem.number
        _mem, _name, editable = self._mem_objs(number)
        if _mem is None:
            raise errors.RadioError(
                f"Channel {number} is outside the writable range for "
                "this experimental driver")
        if _name is not None:
            newname = [str(c) for c in mem.name][:6]
            _name.name = "".join(newname).ljust(6, '\x00')
            # Also clear the 5 pad bytes after the name -- if a name is
            # exactly 6 characters (no null terminator within the name
            # field itself), the official CPS software keeps reading
            # past it looking for a null and displays leftover garbage
            # from these pad bytes (confirmed: "NOTONE" showed as
            # "NOTONEyyy" in the vendor's own software until fixed).
            _name.pad.fill_raw(b'\x00')

        if mem.empty:
            _mem.set_raw(b'\xff' * 13 + b'\x06\x11\x00')
            return

        # Start from the same clean baseline as a freshly-blanked
        # channel (bitfield1=0x06, bitfield2=0x11, mem_changed=0x00),
        # THEN apply real values on top. Building a channel from
        # scratch (e.g. a bulk CSV import) leaves mem.extra empty --
        # generic source radios don't know about this model's
        # busy_lock/ptt_id/signal/freq_hop settings -- so without this
        # baseline those bits are whatever was already in the target
        # record (often all-0xFF/maxed-out garbage from an unrelated
        # prior state), which the radio's firmware doesn't like.
        _mem.set_raw(b'\xff\xff\xff\xff\xff\xff\xff\xff'
                     b'\x00\xff\xff\xff\xff\x06\x11\x00')

        _mem.rx_freq = mem.freq // 10

        if mem.duplex == "+":
            _mem.tx_freq = (mem.freq + mem.offset) // 10
        elif mem.duplex == "-":
            _mem.tx_freq = (mem.freq - mem.offset) // 10
        elif mem.duplex == "":
            _mem.tx_freq = _mem.rx_freq
        elif mem.duplex == "off":
            _mem.tx_freq.fill_raw(b'\xff')

        _mem.mode = self.idx_or_def(mem.mode, self.VALID_MODES)
        _mem.power = self._closest_power_idx(mem.power)
        _mem.scan = False if mem.skip == "S" else True

        ((txmode, txval, txpol),
         (rxmode, rxval, rxpol)) = chirp_common.split_tone_encode(mem)
        self.set_tone(_mem.tx_tone, txmode, txval, txpol)
        self.set_tone(_mem.rx_tone, rxmode, rxval, rxpol)

        for setting in mem.extra:
            setattr(_mem, setting.get_name(), setting.value)
