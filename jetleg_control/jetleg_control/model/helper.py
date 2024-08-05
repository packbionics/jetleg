# Copyright 2023 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import matplotlib.pyplot as plt


def init_subplots(num_plots, subplot_description):
    fig, ax = plt.subplots(num_plots)

    if num_plots == 1:
        subplot_title = subplot_description[0] + \
            ' vs. ' + subplot_description[1]

        ax.set_title(subplot_title)

        ax.set_xlabel(subplot_description[1])
        ax.set_ylabel(subplot_description[0])

        return fig, ax

    for i in range(num_plots):
        subplot_title = subplot_description[i][0] + \
            ' vs. ' + subplot_description[i][1]

        ax[i].set_title(subplot_title)

        ax[i].set_xlabel(subplot_description[i][1])
        ax[i].set_ylabel(subplot_description[i][0])

    return fig, ax


def plot(data_plots, fig, ax):
    # display.clear_output(wait=True)
    # display.display(plt.gcf())
    # plt.clf()

    fig.suptitle('Training...')

    num_plots = len(data_plots)
    if num_plots == 1:
        data = data_plots[0]
        ax.plot(data)
    else:
        for i in range(num_plots):
            data = data_plots[i]
            ax[i].plot(data)

    plt.pause(0.1)
