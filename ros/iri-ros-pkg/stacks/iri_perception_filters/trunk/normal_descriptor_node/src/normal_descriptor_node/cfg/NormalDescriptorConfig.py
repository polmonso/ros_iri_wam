## *********************************************************
## 
## File autogenerated for the normal_descriptor_node package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

config_description = [{'srcline': 43, 'description': 'Number of spatial bins per side (not implemented yet)', 'max': 1, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/normal_descriptor_alg_config.cfg', 'name': 'num_spatial_bins', 'edit_method': '', 'default': 1, 'level': 1, 'min': 1, 'type': 'int'}, {'srcline': 44, 'description': 'Number of orientation bins per side', 'max': 128, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/normal_descriptor_alg_config.cfg', 'name': 'num_orientation_bins', 'edit_method': '', 'default': 8, 'level': 1, 'min': 16, 'type': 'int'}, {'srcline': 45, 'description': 'Number of pixels for the local patch radius', 'max': 65, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/normal_descriptor_alg_config.cfg', 'name': 'desc_patch_radius', 'edit_method': '', 'default': 7, 'level': 1, 'min': 15, 'type': 'int'}, {'srcline': 46, 'description': 'spacing in pixels between each sampled position', 'max': 32, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '../cfg/normal_descriptor_alg_config.cfg', 'name': 'sample_each', 'edit_method': '', 'default': 1, 'level': 1, 'min': 1, 'type': 'int'}]

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

for param in config_description:
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']
