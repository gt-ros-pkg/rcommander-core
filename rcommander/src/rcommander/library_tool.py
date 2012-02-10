import tool_utils as tu
from PyQt4.QtGui import *
from PyQt4.QtCore import *

import os
import glob
import os.path as pt
import cPickle as pk
import shutil as su
import graph_model as gm

#
# Library nodes saved in ~/.rcommander/library/
#
class LibraryTool(tu.ToolBase):

    def __init__(self, button, rcommander):
        tu.ToolBase.__init__(self, rcommander, 'library', 'Library', 'library')
        self.button = button
        self.rcommander.connect(self.button, SIGNAL('clicked()'), self.activate_cb)

    def _create_tree_model(self):
        tree_model = QStandardItemModel()
        tree_model.setHorizontalHeaderLabels(['Saved States'])
        root_node = tree_model.invisibleRootItem()

        #Traverse directory looking for saved states
        state_names = glob.glob(pt.join(self._get_library_home(), '*.state'))
        slist_dict = {}
        for sfilename in state_names:
            sfile = open(sfilename, 'r')
            loaded_state = pk.load(sfile)
            sfile.close()

            class_name = loaded_state.__class__.__name__
            if not slist_dict.has_key(class_name):
                slist_dict[class_name] = {}
            slist_dict[class_name][loaded_state.get_name()] = {'file': sfilename, 'state': loaded_state}
            #slist_dict[class_name].append([sfilename, loaded_state])

        all_classes = slist_dict.keys()
        all_classes.sort()
        #print 'all_classes', all_classes
        for n in all_classes:
            item = QStandardItem(n)
            item.setEditable(False)
            item.setSelectable(False)
            state_names = slist_dict[n].keys()
            state_names.sort()
            slist_dict[n]['__qstandard_item__'] = item
            #print n, slist_dict[n].keys()
            for state_name in state_names:
                state_item = QStandardItem(state_name)
                state_item.setEditable(False)
                item.appendRow(state_item)
            root_node.appendRow(item)

        return tree_model, root_node, slist_dict

    def edit_cb(self, index):
        print 'edit called', index.row()

    def fill_property_box(self, pbox):
        #Remove everything from parent container
        container = self.rcommander.ui.properties_container
        layout = container.layout()
        for i in range(layout.count()):
            item = layout.itemAt(0)
            layout.removeItem(item)
        children = container.children()
        for c in children[1:]:
            try:
                layout.removeWidget(c)
            except TypeError, e:
                pass

        layout.invalidate()
        container.update()

        #delete layout
        clayout = container.layout()
        clayout.deleteLater()
        QCoreApplication.sendPostedEvents(clayout, QEvent.DeferredDelete)

        #Create a new layout
        clayout = QVBoxLayout(container)
        clayout.setMargin(0)

        self.tree_view = QTreeView(container)
        self.tree_model, root_node, self.slist_dict = self._create_tree_model()
        self.tree_view.setModel(self.tree_model)
        self.tree_view.expandAll()
        #self.tree_view.setEditTriggers(QAbstractItemView.AllEditTriggers)
        #self.rcommander.connect(self.tree_view, SIGNAL('edit(QModelIndex)'), self.edit_cb)

        self.button_panel = QWidget(container)
        bpanel_layout = QHBoxLayout(self.button_panel)
        bpanel_layout.setMargin(0)

        self.delete_button = QPushButton(self.button_panel)
        self.delete_button.setText('Delete')
        bpanel_layout.addWidget(self.delete_button)
        self.rcommander.connect(self.delete_button, SIGNAL('clicked()'), self.delete_cb)

        clayout.addWidget(self.tree_view)
        clayout.addWidget(self.button_panel)

        pidx = self.rcommander.ui.node_settings_tabs.indexOf(self.rcommander.ui.properties_container)
        self.rcommander.ui.node_settings_tabs.setCurrentIndex(pidx)

    def fill_connections_box(self, pbox):
        self.name_input = QLineEdit()
        return

    def new_node(self, name = None):
        #Get the current selected state, make a clone of it
        index = self.tree_view.selectionModel().currentIndex()

        #assume that the correct index is selected always
        name_of_state = str(index.data(Qt.DisplayRole).toString())
        name_of_class = str(index.parent().data(Qt.DisplayRole).toString())
        if name_of_state == '':
            return None

        state_file = open(self.slist_dict[name_of_class][name_of_state]['file'], 'r')
        state = pk.load(state_file)
        state_file.close()

        return state

    def set_node_properties(self, my_node):
        pass

    def reset(self):
        pass

    def _get_library_home(self):
        #create the library if it does not exist
        rcommander_home = pt.join(os.getenv("HOME"), '.rcommander')
        library_home = pt.join(rcommander_home, 'library')

        #if not pt.exists(rcommander_home):
        #    os.mkdir(rcommander_home)

        if not pt.exists(library_home):
            os.makedirs(library_home)

        return library_home

    #Special method to this tool, called by rcommander.
    def add_to_library(self, state_node):
        library_home = self._get_library_home()

        if gm.is_container(state_node):
            state_node.save_child(library_home)

        #if there's a node in the library of the same name.
        state_fname = pt.join(library_home, "%s.state" % (state_node.get_name()))
        i = 0
        while pt.exists(state_fname):
            state_fname = pt.join(library_home, "%s_%d.state" % (state_node.get_name(), i))
            i = i + 1

        #Save!
        state_file = open(state_fname, 'w')
        pk.dump(state_node, state_file)
        state_file.close()

        #Recreate everything in the GUI!
        self.activate_cb()

    def delete_cb(self):
        index = self.tree_view.selectionModel().currentIndex()
        name_of_state = str(index.data(Qt.DisplayRole).toString())
        name_of_class = str(index.parent().data(Qt.DisplayRole).toString())
        file_name = self.slist_dict[name_of_class][name_of_state]['file']

        trash_home = pt.join(os.getenv("HOME"), '.Trash')
        if not pt.exists(trash_home):
            os.mkdir(trash_home)

        su.move(file_name, pt.join(trash_home, pt.split(file_name)[1]))
        #Recreate everything in the GUI!
        self.activate_cb()


