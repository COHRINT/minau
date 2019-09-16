#!/usr/bin/env python

from __future__ import division

"""
Class implementation of quantization algorithm.
"""

import os
import math
import yaml
import numpy as np
from decimal import Decimal, getcontext
getcontext().prec = 500

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
            num_bins_element = Decimal(measurement_range[i][1]-measurement_range[i][0])*(Decimal(10**(-1*np.log10(measurement_resolution[i]))))
            num_bins.append(num_bins_element)
        # num_bins = [int(num_bins) for i in range(0,len(measurement))]

        # compute bin for each element
        bin_list = self.vals2bins(measurement,measurement_range,measurement_resolution)

        # compute config number
        config_num = self.bin2config(bin_list,num_bins)

        # convert to binary
        bits = bin(int(config_num))

        return_vals = []
        if bits_flag: return_vals.append(bits)
        if config_flag: return_vals.append(config_num)
        
        return return_vals

    def quant2meas(self,bitstring,num_els,type_=None,measurement_range=None,measurement_resolution=None,config_num=None):
        """
        Convert quantized measurement to measurement using range and resolution.
        """
        # convert from bits to configuration number
        if config_num is None:
            config_num = long(bitstring,base=2)
        config_num = Decimal(config_num)

        # retrieve measurement parameters
        if type_ in self.cfg['meas_types']:
            element_types = self.cfg['meas_types'][type_]
            measurement_range = [self.cfg[element]['range'] for element in element_types]
            measurement_resolution = [self.cfg[element]['resolution'] for element in element_types]

        # compute number of bins --> note that instead of dividing by the resolution,
        # we use the following method to avoid floating point error
        num_bins = []
        for i in range(0,len(measurement_range)):
            num_bins_element = Decimal(measurement_range[i][1]-measurement_range[i][0])*(Decimal(10**(-1*np.log10(measurement_resolution[i]))))
            num_bins.append(num_bins_element)
        # num_bins = [int(num_bins) for i in range(0,num_els)]

        # compute bin numbers
        bin_num_list = self.config2bins(config_num,num_els,num_bins)

        # compute element values from bin numbers
        measurement_list = self.bins2vals(bin_num_list,measurement_resolution,measurement_range)

        return measurement_list

    def state2quant(self,mean_vec,cov,element_types,mean_range=None,mean_resolution=None,diag_range=None,
        diag_resolution=None,offdiag_range=None,offdiag_resolution=None,bits_flag=True,config_flag=False):
        """
        Convert state estimate to quantized representation using config values or 
        passed ranges and resolutions.

        Parameters
        ----------
        mean_vec
            mean vector of state estimate -- n x 1 numpy array
        cov
            full covariance matrix of estimate -- n x n numpy array
        element types
            n element list of estimate state element types, e.g. ['position','velocity']
        mean_range : [default None]
            range of values for mean vector
        mean_resolution : [default None]

        diag_range : [default None]

        diag_resolution : [default None]

        offdiag_range : [default None]

        offdiag_resolution : [default None]

        bits_flag : [default True]
            return bitstring of quantized state estimate
        config_flag : [default False]
            return integer config number of quantized state estimate
        """
        # get ranges and resolutions from config
        mean_range = [self.cfg[element]['range'] for element in element_types]
        mean_resolution = [self.cfg[element]['resolution'] for element in element_types]
        
        diag_range = [self.cfg[element]['variance_range'] for element in element_types]
        diag_resolution = [self.cfg[element]['variance_resolution'] for element in element_types]

        offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
        offdiag_resolution = [self.cfg['covar_offdiag_resolution'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
        # first compute number of bins
        mean_num_bins = []
        for i in range(0,len(mean_range)):
            mean_bins_element = int((mean_range[i][1]-mean_range[i][0])*(10**(-1*np.log10(mean_resolution[i]))))
            mean_num_bins.append(mean_bins_element)

        diag_num_bins = []
        for i in range(0,len(diag_range)):
            diag_bins_element = int((diag_range[i][1]-diag_range[i][0])*(10**(-1*np.log10(diag_resolution[i]))))
            diag_num_bins.append(diag_bins_element)

        offdiag_num_bins = []
        for i in range(0,len(offdiag_range)):
            offdiag_bins_element = int((offdiag_range[i][1]-offdiag_range[i][0])*(10**(-1*np.log10(offdiag_resolution[i]))))
            offdiag_num_bins.append(offdiag_bins_element)

        # extract diagonal cov elements
        diag_els = np.diag(cov)
        # extract off-diagonal, upper-triangular elements
        offdiag_els = np.extract(np.triu(1-np.eye(cov.shape[0])),cov)

        # combine both elements and max number of bin lists of mean, diagonal, and off-diagonal
        elements = np.concatenate((mean_vec,diag_els,offdiag_els))
        element_resolution = mean_resolution + diag_resolution + offdiag_resolution
        element_range = mean_range + diag_range + offdiag_range
        num_bins = mean_num_bins + diag_num_bins + offdiag_num_bins

        bin_list = self.vals2bins(elements,element_range,element_resolution)

        config_num = self.bin2config(bin_list,num_bins)

        # convert to bitstring
        bits = bin(long(config_num))

        return_vals = []
        if bits_flag: return_vals.append(bits)
        if config_flag: return_vals.append(config_num)
        
        return return_vals

    def quant2state(self,bitstring,num_els,element_types,mean_range=None,mean_resolution=None,
                    diag_range=None,diag_resolution=None,offdiag_range=None,offdiag_resolution=None,config_num=None):
        """
        Convert quantized state to state using config values or passed ranges and resolutions.

        Parameters
        ----------
        bitstring
            bitstring of quantized measurements
        num_els
            number of elements to convert to values
        element_types
            state element types
        mean_range : [default None]
            range of values for mean vector
        mean_resolution : [default None]

        diag_range : [default None]

        diag_resolution : [default None]

        offdiag_range : [default None]

        offdiag_resolution : [default None]

        config_num : [default False]
            use integer config number instead of bistring
        """
        # convert from bits to configuration number
        if config_num is None:
            config_num = int(bitstring,base=2)
        config_num = Decimal(value=config_num)

        # get ranges and resolutions from config
        mean_range = [self.cfg[element]['range'] for element in element_types]
        mean_resolution = [self.cfg[element]['resolution'] for element in element_types]
        
        diag_range = [self.cfg[element]['variance_range'] for element in element_types]
        diag_resolution = [self.cfg[element]['variance_resolution'] for element in element_types]

        offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
        offdiag_resolution = [self.cfg['covar_offdiag_resolution'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

        # first compute number of bins
        mean_num_bins = []
        for i in range(0,len(mean_range)):
            mean_bins_element = int((mean_range[i][1]-mean_range[i][0])*(10**(-1*np.log10(mean_resolution[i]))))
            mean_num_bins.append(mean_bins_element)

        diag_num_bins = []
        for i in range(0,len(diag_range)):
            diag_bins_element = int((diag_range[i][1]-diag_range[i][0])*(10**(-1*np.log10(diag_resolution[i]))))
            diag_num_bins.append(diag_bins_element)

        offdiag_num_bins = []
        for i in range(0,len(offdiag_range)):
            offdiag_bins_element = int((offdiag_range[i][1]-offdiag_range[i][0])*(10**(-1*np.log10(offdiag_resolution[i]))))
            offdiag_num_bins.append(offdiag_bins_element)

        # combine both elements and max number of bin lists of mean, diagonal, and off-diagonal
        element_resolution = mean_resolution + diag_resolution + offdiag_resolution
        element_range = mean_range + diag_range + offdiag_range
        num_bins = mean_num_bins + diag_num_bins + offdiag_num_bins

        # compute bin numbers
        bin_num_list = self.config2bins(config_num, num_els, num_bins)

        # compute values
        values = self.bins2vals(bin_num_list,element_resolution,element_range)

        # recover mean and covariance
        mean_vec = np.array(values[:len(element_types)])
        mean_vec = np.atleast_2d(mean_vec)

        diag_vec = values[len(element_types):2*len(element_types)]
        cov = np.diag(diag_vec)

        offdiag_vec = values[2*len(element_types):]
        off_diag_els_u = np.triu(1-np.eye(cov.shape[0]))
        off_diag_els_u = off_diag_els_u.astype(bool)
        off_diag_els_l = np.tril(1-np.eye(cov.shape[0]))
        off_diag_els_l = off_diag_els_l.astype(bool)
        cov[off_diag_els_u] = offdiag_vec
        cov[off_diag_els_l] = offdiag_vec

        return mean_vec, cov

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
                print(i,el)

            # determine rounding method (depends on if resolution is smaller than 1)
            if resolution[i] < 1:
                # shift range to 0, then round to resolution
                rounded = round(el-range_[i][0],int(abs(np.log10(resolution[i]))))
                bin_num = rounded * (10**(-1*np.log10(resolution[i])))
                bin_num_list.append((int(bin_num)))
        
        return bin_num_list

    def bin2config(self, bin_num_list, num_bins):
        """
        Compute configuration number from list of element bin numbers and total
        number of possible bins for each element.
        """
        config_num = Decimal(bin_num_list[0])

        for i in range(1,len(bin_num_list)):
            prod_val = Decimal(1)
            for j in range(1,i+1):
                prod_val *= Decimal(num_bins[j-1])
            config_num += prod_val*(Decimal(bin_num_list[i])-Decimal(1))

        return config_num

    def config2bins(self, config_num, num_els, num_bins):
        """
        Convert from received configuration number to bin numbers.
        """
        bin_num_list = []
        for i in range(1,num_els+1):
            if i != num_els:
                bin_prod = Decimal(1)
                for j in range(0,len(num_bins)-i):
                    bin_prod *= Decimal(num_bins[j])
                bin_num = Decimal(math.floor(config_num/bin_prod)) + Decimal(1)
                config_num = config_num - bin_prod*(Decimal(bin_num)-Decimal(1))
            else:
                bin_num = config_num
            bin_num_list.append(bin_num)

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

    print('---------------------------------')

    # state1_mean = [-0.89, 0.2345,0.9]
    # state1_cov = np.array([[1,0.1,-0.674],
    #                 [0.1,1.5,0.216],
    #                 [-0.674,0.216,1.75]])
    # element_types = ['position','velocity','angle']
    
    # state1_mean = [103.10290923,5.1093]
    # state1_cov = np.array([[1003,0.030982],
    #                         [0.030982,25.21098]])
    # element_types = ['position','velocity']

    state1_mean = [-0.89, 0.2345,10.20912,1.302,-3.4311,-0.5922]
    state1_cov = np.array([ [1,0.1,-0.674,0.2198,-0.129,1.1901],
                            [0.1,10.5,0.291,0.2198,0.1,0.1],
                            [-0.674,0.291,20.10909,0.2198,-0.129,1.1901],
                            [0.2198,0.2198,0.2198,5.292,-0.129,0.2198],
                            [-0.129,0.1,-0.129,-0.129,50.309,1.1901],
                            [1.1901,0.1,0.2198,0.2198,1.1901,3.290]])
    element_types = ['position','velocity','position','velocity','position','velocity']

    print(state1_mean)
    print(state1_cov)

    bits = q.state2quant(state1_mean,state1_cov,element_types)
    print(bits[0])
    print(len(bits[0]))
    num_els = int(len(element_types) + 0.5*len(element_types)*(len(element_types)+1))
    mean,cov = q.quant2state(bits[0],num_els,element_types)
    print(mean)
    print(cov)