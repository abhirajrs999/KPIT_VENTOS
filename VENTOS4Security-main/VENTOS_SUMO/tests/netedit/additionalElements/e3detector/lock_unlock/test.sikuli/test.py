# import common functions for netedit tests
import os
import sys

testRoot = os.path.join(os.environ.get('SUMO_HOME', '.'), 'tests')
neteditTestRoot = os.path.join(
    os.environ.get('TEXTTEST_HOME', testRoot), 'netedit')
sys.path.append(neteditTestRoot)
import neteditTestFunctions as netedit

# Open netedit
neteditProcess, match = netedit.setupAndStart(neteditTestRoot, False)

# apply zoom
netedit.zoomIn(match.getTarget().offset(325, 200), 10)

# Change to create additional
netedit.additionalMode()

# obtain match for comboboxAdditional
comboboxAdditional = netedit.getComboBoxAdditional(match)

# select E3
netedit.changeAdditional(comboboxAdditional, 8)

# create E3
netedit.leftClick(match, 250, 250)

# select entry detector
netedit.changeAdditional(comboboxAdditional, -4)

# Create Entry detector E3 (for saving)
netedit.selectChild(comboboxAdditional, 4, 1)
netedit.leftClick(match, 400, 250)

# change to move mode
netedit.moveMode()

# move E3 to up
netedit.moveElement(match, 120, 250, 120, 50)

# Change to inspect mode
netedit.inspectMode()

# inspect E3
netedit.leftClick(match, 250, 100)

# obtain parameters reference
parametersReference = netedit.getParametersReference(match)

# block additional
netedit.changeBlockAdditional(comboboxAdditional, 7)

# change to move mode
netedit.moveMode()

# try to move E3 to right (must be blocked)
netedit.moveElement(match, 120, 50, 300, 50)

# Change to inspect mode
netedit.inspectMode()

# inspect E3
netedit.leftClick(match, 250, 100)

# obtain parameters reference
parametersReference = netedit.getParametersReference(match)

# unblock additional
netedit.changeBlockAdditional(comboboxAdditional, 7)

# change to move mode
netedit.moveMode()

# move E3 to right (must be allowed)
netedit.moveElement(match, 120, 50, 300, 50)

# Check undos and redos
netedit.undo(match, 6)
netedit.redo(match, 6)

# save additionals
netedit.saveAdditionals()

# save newtork
netedit.saveNetwork()

# quit netedit
netedit.quit(neteditProcess, False, False)
