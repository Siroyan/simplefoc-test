"""Keep comparisons outside the commanded reference ramp."""
import re


def reference_duration_us(lines):
    line=next((line for line in lines if line.startswith('REFERENCE_RAMP ')),None)
    if line is None:
        return 0
    match=re.search(r'duration_us=(\d+)(?: |$)',line)
    if not match or not 0<int(match[1])<3000000:
        raise ValueError('Invalid or out-of-run reference ramp duration')
    return int(match[1])


def comparison_start_us(lines, requested=None):
    minimum=max(800000,reference_duration_us(lines))
    if requested is None:
        return minimum
    if requested<minimum or requested>=3000000:
        raise ValueError('Comparison start must follow startup and reference ramp, inside the run')
    return requested
