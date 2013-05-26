#!/usr/bin/env python

import re
from param import *
from emit import Emit

# Emit docs in a form acceptable to the APM wordpress docs site
class HtmlEmit(Emit):
    
    def __init__(self):
        html_fname = 'Parameters.html'
        self.f = open(html_fname, mode='w')
        self.preamble = '''<!-- Dynamically generated list of documented parameters
This page was generated using Tools/autotest/param_metadata/param_parse.py

DO NOT EDIT
-->

<!-- add auto-generated table of contents with "Table of Contents Plus" plugin -->
[toc]\n
'''        
        self.t = ''

    def escape(self, s):
        s = s.replace(' ', '-')
        s = s.replace(':', '-')
        s = s.replace('(', '')
        s = s.replace(')', '')
        return s

    def close(self):
        self.f.write(self.preamble)
        self.f.write(self.t)
        self.f.close()
    
    def start_libraries(self):
        self.t += '\n\n<h1>Libraries</h1>\n\n'
        
    def emit(self, g, f):    
        tag = '%s Parameters' % g.name
        t = '\n\n<h1>%s</h1>\n'  % tag
        
        for param in g.params:
            if not hasattr(param, 'DisplayName') or not hasattr(param, 'Description'):
                continue
            tag = '%s (%s)' % (param.DisplayName, param.name)
            t += '\n\n<h2>%s</h2>' % tag
            t += "\n\n<p>%s</p>\n" % param.Description
            t += "<ul>\n"
            
            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    if field == 'Values' and Emit.prog_values_field.match(param.__dict__[field]):
                        values = (param.__dict__[field]).split(',')
                        t += "<table><th>Value</th><th>Meaning</th>\n"
                        for value in values:
                            v = value.split(':')
                            t += "<tr><td>%s</td><td>%s</td></tr>\n" % (v[0], v[1])
                        t += "</table>\n"
                    else:
                        t += "<li>%s: %s</li>\n" % (field, param.__dict__[field])
            t += "</ul>\n"
        self.t += t

