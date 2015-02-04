def plot_xy_coordinates(ax, xs, ys, param_dict={}):
    return ax.plot(xs, ys, **param_dict)


def label_axes(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
