#!/usr/bin/env python

from __future__ import division

"""
Class implementation of quantization algorithm.
"""

import os
import math
import yaml
import numpy as np
from decimal import Decimal

class Quantizer:
    """
    Class implementation of measurement and state quantization algorithm.
    Compresses floating point data for transmission in severely bandwidth-
    limited applications.

    Exposes interfaces for transmitting measurements and states, as well as
    a general purpose string of data.

    Parameters
    ----------
    quantizer_fxn
        string name of desired quantization function, e.g. uniform, x^3 binning, etc.
        Fxns: uniform, x3 (f(x)=x^3), x5 (f(x) = x^5)
    config_path
        path to yaml config file specifying quantization function parameters and
        measurement and state encoding parameters.
    """
    def __init__(self,quantizer_fxn='uniform',config_path='../config/quantization_config.yaml'):
        
        # ensure config path is absolute
        config_path = os.path.abspath(config_path)
        # load config
        with open(config_path,'r') as f:
            self.cfg = yaml.load(f)

        # check that quant fxn is allowable type
        try:
            assert(quantizer_fxn in self.cfg['quant_types'])
        except AssertionError:
            print('Passed quantization function is not implemented type!')
            print('Available types: {}'.format(self.cfg['quant_types']))
            print('Falling back to uniform quantization.')

        # number of available bits for message
        self.available_bits = self.cfg['available_bits']

    def meas2quant(self,measurement,type_=None,measurement_range=None,measurement_resolution=None,bits_flag=True,config_flag=False):
        """
        Convert measurement into quantized representation using passed range and resolution.

        Parameters
        ----------
        measurement
            measurement data to be transmitted
        type_ : [default - None]
            measurement type for parameter lookup in quantization config
        measurement_range : [default - None]
            list of measurement element ranges (length=# of measurement elements)
            Note: not used if type_ is passed
        measurement_resolution : [default - None]
            list of measurement element resolutions (length=# of measurement elements)
            Note: not used if type_ is passed
        bits_flag: [default - True]
            return the bitstring of the quantized measurement
        config_flag: [default - False]
            return the configuration number before conversion to bitstring
        """
        # retrieve measurement parameters
        if type_ in self.cfg['meas_types']:
            # measurement_range = self.cfg['meas_types'][type_]['range']
            # measurement_resolution = self.cfg['meas_types'][type_]['resolution']
            element_types = self.cfg['meas_types'][type_]
            measurement_range = [self.cfg[element]['range'] for element in element_types]
            measurement_resolution = [self.cfg[element]['resolution'] for element in element_types]

        # compute number of bins --> note that instead of dividing by the resolution,
        # we use the following method to avoid floating point error
        num_bins = []
        for i in range(0,len(measurement_range)):
            num_bins_element = int((measurement_range[i][1]-measurement_range[i][0])*(10**(-1*np.log10(measurement_resolution[i]))))
            num_bins.append(num_bins_element)
        # num_bins = [int(num_bins) for i in range(0,len(measurement))]

        # compute bin for each element
        bin_list = self.vals2bins(measurement,measurement_range,measurement_resolution)

        # compute config number
        config_num = self.bin2config(bin_list,num_bins)
        print(config_num)

        # convert to binary
        bits = bin(long(config_num)) 

        return_vals = []
        if bits_flag: return_vals.append(bits)
        if config_flag: return_vals.append(config_num)
        
        return return_vals

    def quant2meas(self,bitstring,num_els,type_=None,measurement_range=None,measurement_resolution=None,config_num=None):
        """
        Convert quantized measurement to measurement using range and resolution.
        """
        # retrieve measurement parameters
        if type_ in self.cfg['meas_types']:
            element_types = self.cfg['meas_types'][type_]
            measurement_range = [self.cfg[element]['range'] for element in element_types]
            measurement_resolution = [self.cfg[element]['resolution'] for element in element_types]

        # convert from bits to configuration number
        if config_num is None:
            config_num = long(bitstring,base=2)
        config_num = Decimal(config_num)

        # compute number of bins --> note that instead of dividing by the resolution,
        # we use the following method to avoid floating point error
        num_bins = []
        for i in range(0,len(measurement_range)):
            num_bins_element = int((measurement_range[i][1]-measurement_range[i][0])*(10**(-1*np.log10(measurement_resolution[i]))))
            num_bins.append(num_bins_element)
        # num_bins = [int(num_bins) for i in range(0,num_els)]

        # compute bin numbers
        bin_num_list = self.config2bins(config_num,num_els,num_bins)

        # compute element values from bin numbers
        measurement_list = self.bins2vals(bin_num_list,measurement_resolution,measurement_range)

        return measurement_list

    def state2quant(self,state_est,cov,est_range,est_resolution,cov_diag_range,
        cov_diag_resolution,cov_offdiag_range,cov_offdiag_resolution):
        """
        Convert state estimate to quantized representation using passed ranges and resolutions.
        """
        # first compute number of bins
        state_bins = (est_range[1]-est_range[0])*(10**(-1*np.log10(est_resolution)))
        diag_bins = (cov_diag_range[1]-cov_diag_range[0])*(10**(-1*np.log10(cov_diag_resolution)))
        offdiag_bins = (cov_offdiag_range[1]-cov_offdiag_range[0])*(10**(-1*np.log10(cov_offdiag_resolution)))

        # extract diagonal cov elements
        diag_els = np.diag(cov)
        # extract off-diagonal, upper-triangular elements
        offdiag_els = np.extract(np.triu(1-np.eye(cov.shape[0])),cov)

        # compute bin number for each state element
        state_bin_list = self.vals2bins(state_est,est_range,est_resolution)
        diag_bin_list = self.vals2bins(diag_els,cov_diag_range,cov_diag_resolution)
        offdiag_bin_list = self.vals2bins(offdiag_els,cov_offdiag_range,cov_offdiag_resolution)

        # create full list of all bin numbers, and list of corresponding resolutions
        state_bin_num = state_bins*np.ones_like(state_bin_list) 
        diag_bin_num = diag_bins*np.ones_like(diag_bin_list)
        offdiag_bin_num = offdiag_bins*np.ones_like(offdiag_bin_list)
        bin_num = np.concatenate((state_bin_num,diag_bin_num,offdiag_bin_num))

        bin_list = np.concatenate((state_bin_list,diag_bin_list,offdiag_bin_list))

        config_num = self.bin2config(bin_list,bin_num)

        # convert to bitstring
        bits = bin(long(config_num))

        return bits

    def quant2state(self):
        pass

    def vals2bins(self, vals, range_, resolution):
        """
        Compute the bin numbers for a quantized version of vals at given range and resolution.
        """
        bin_num_list = []
        for i,el in enumerate(vals):
            # make sure element is within range
            if el < range_[i][0]:
                el = range_[i][0]
                print('Warning: Element is outside allowable range, setting to min.')
            elif el > range_[i][1]:
                el = range_[i][1]
                print('Warning: Element is outside allowable range, setting to max.')

            # determine rounding method (depends on if resolution is smaller than 1)
            if resolution[i] < 1:
                # shift range to 0, then round to resolution
                rounded = round(el-range_[i][0],int(abs(np.log10(resolution[i]))))
                # print('rounded: {}'.format(rounded))
                bin_num = rounded * (1/resolution[i])
                bin_num_list.append((int(bin_num)))
        
        print(bin_num_list)
        return bin_num_list

    def bin2config(self, bin_num_list, num_bins):
        """
        Compute configuration number from list of element bin numbers and total
        number of possible bins for each element.
        """
        config_num = bin_num_list[0]

        for i in range(1,len(bin_num_list)):
            prod_val = 1
            for j in range(1,i+1):
                prod_val *= num_bins[j-1]
            config_num += prod_val*(bin_num_list[i]-1)

        return config_num

    def config2bins(self, config_num, num_els, num_bins):
        """
        Convert from received configuration number to bin numbers.
        """
        # reverse order of max bin sizes
        # num_bins.reverse()
        print(num_bins)

        bin_num_list = []
        for i in range(1,num_els+1):
            if i != num_els:
                bin_prod = Decimal(1)
                for j in range(0,len(num_bins)-i):
                    print(i)
                    print(num_bins[j])
                    bin_prod *= Decimal(num_bins[j])
                bin_num = Decimal(math.floor(config_num/bin_prod)) + Decimal(1)
                config_num = config_num - bin_prod*(Decimal(bin_num)-Decimal(1))
            else:
                bin_num = config_num
            bin_num_list.append(bin_num)

        print(bin_num_list)
        return bin_num_list

    def bins2vals(self, bins, val_resolution, val_range):
        """
        Convert from bin numbers to element values.

        Parameters
        ----------
        bins
            list of bin numbers for each element of encoded message
        val_resolution
            resolution of elements
        val_range
            range of element values
        """
        # reverse resolution and range
        val_resolution.reverse()
        val_range.reverse()

        elements = []
        for i in range(0,len(bins)):
            el_val = bins[i]*Decimal(val_resolution[i]) + Decimal(val_range[i][0])

            elements.append(float(el_val))

        elements.reverse()

        return elements

if __name__ == "__main__":
    q = Quantizer()

    meas = [100.289, 0.439012, 1.209123]
    # meas = [30.2128979]
    print(meas)
    meas_type = 'usbl'

    bits = q.meas2quant(meas,type_=meas_type)
    print(bits[0])
    meas_decoded = q.quant2meas(bits[0],len(meas),type_=meas_type)
    print(meas_decoded)