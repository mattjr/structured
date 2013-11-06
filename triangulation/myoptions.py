##################################################
# Copyright (c) INRIA (France) 2011, 2012, 2013
# 
# This file is part of inria-mvs. You can redistribute it and/or
# modify it under the terms of the GNU General Public License.
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
# 
# Author: Jean-Paul CHIEZE <jean-paul.chieze@inria.fr>
# 
##################################################

import re, os, sys

global cur_doc
cur_doc = "";

def sphinxdoc_of_options(options):
    """
    Build a doc string in sphinx format from options list

    Input : 
      - options = list of option definitions (see parse_args below)
    Returns:
      tuple made of usage line and concatenation  of detailed options help strings.
    """
    args = []
    argsdoc = []
    for (opt,nbargs,required,argname,comm) in options:
        if comm != "":
            if opt == None:
                argsdoc.append("\t* **%s**: %s" %(argname,comm))
            else:
                if (nbargs > 0):
                    argsdoc.append("\t* **%s** %s: %s" %(opt,argname,comm))
                else:
                    argsdoc.append("\t* **%s**: %s" %(opt,comm))
        if opt == None:
            s = "**%s**" %argname
        else:
            if (nbargs > 0):
                s = "**%s** *%s*" %(opt,argname)
            else:
                s = "**%s**" %opt
        if required:
            args.append(s)
        else:
            args.append("[%s]" %s)
    return (" ".join(args),"\n".join(argsdoc) + "\n")

def doc_of_options(options):
    """
    Build a doc string from options list (removes some  rst tags)

    Input : 
      - options = list of option definitions (see parse_args below)
    Returns:
      tuple made of usage line and concatenation  of detailed options help strings.
    """
    (txt,doc) = sphinxdoc_of_options(options)
    rexp = re.compile('\*')
    return (re.sub(rexp,'',txt),re.sub(rexp,'',doc))

def prepare_doc(options,before = "\n",after= "",name = None):
    """
    Returns a string mde of options documentation with optional strings 
    prepended and appended.
    The resulting string is stored in a global variable for use by usage()
    """
    global cur_doc
    (txt,ldoc) = sphinxdoc_of_options(options)
    sname = None
    if name == '__main__' or name == None:
        sname = os.path.basename(sys.argv[0])
    if sname == '' or sname == None:
        sname = name
    if sname == None:
        doc = "%s\n" %txt
    else:
        doc = "**%s** %s\n" %(sname,txt)
    doc = doc + before + ldoc + after
    cur_doc = doc
    return doc

def usage(msg = None):
    """
    Prints the optional <msg> string, followed by
    the <cur_doc> string, that must have been built with prepare_doc
    """
    if(msg != None):
        print msg
    rexp = re.compile('\*')
    rexp2 = re.compile('\n\|')
    s = re.sub(rexp,'',cur_doc)
    s = re.sub(rexp2,'\n',s)
    print "Usage: ",  s
    sys.exit(1)
    
def parse_args(options,argv,usage = usage):
    """
    Retrieve options from command line arguments (argv), according to a list of tuples (options).
    

    Input:
       - options : a list of 5 elements tuples describing each option
 
         + 1st element : the option string (for ex -i).
           If None, option is only used for making the doc string.
         + 2nd element: number of arguments of the option.
         + 3rd element : True if option is mandatory.
         + 4th element : argument name(s).
         + 5th element : description string.
       - argv : the list of command line arguments. Arguments starting with
           '-' arte considered options and checked against <options>.
       - usage : the usage function to call (myoptions.usage() by default).

    Returns:
      (list of (option-strings,arg-value), list of non options arguments).
    """
    args = []
    opts = []
    h_opts = {}
    found_required = {}
    for (opt,nbargs,required,argname,comm) in options:
        if opt == None:
            continue
        h_opts[opt] = nbargs
        if required:
            found_required[opt] = False
    n = len(argv)
    i = 0
    while i < n:
        opt = argv[i]
        i += 1
        if opt[0] == '-':
            if opt in h_opts:
                if opt in found_required:
                    found_required[opt] = True
                nbargs = h_opts[opt]
                if(nbargs > 0):
                    i2 = i + nbargs - 1
                    if i2 >= n:
                        usage("Missing arg for option %s" %opt)
                    i2 += 1
                    if(nbargs == 1):
                        arg = argv[i]
                    else:
                        arg = argv[i:i2]
                    i = i2
                else:
                    arg = None
            else:
                usage('Bad option %s' %opt)
            opts.append((opt,arg))
        else:
            args.append(opt)
    l = []
    for opt in found_required.keys():
        if (not found_required[opt]):
            l.append(opt)
    if (len(l) != 0):
        usage("The following options are required : %s" %str(l))
    return opts, args

