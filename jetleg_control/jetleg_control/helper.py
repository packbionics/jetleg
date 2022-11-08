import matplotlib.pyplot as plt

# plt.ion()

def init_subplots(num_plots, subplot_description):
    fig, ax = plt.subplots(num_plots)

    for i in range(num_plots):
        subplot_title = subplot_description[i][0] + ' vs. ' + subplot_description[i][1]

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
    for i in range(num_plots):
        data = data_plots[i]

        ax[i].plot(data)
    
    plt.pause(0.1)
