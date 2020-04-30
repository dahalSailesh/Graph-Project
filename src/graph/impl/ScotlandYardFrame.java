package graph.impl;


import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.imageio.ImageIO;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;

import graph.IGraph;

@SuppressWarnings("serial")
public class ScotlandYardFrame extends JFrame
{
    // the canvas contained in the frame where we'll draw everything
    private JPanel canvas;
    
    private JLabel label=new JLabel("");
    
    // Map from location names (such as "1") to their (x, y) coordinates on the screen
    // Use the built-in Point class in Java
    private Map<String,Point> pointMap;
    // the graph
    private IGraph graph;
    // map of moves; key is the move number (i.e. 1, 2, 3, etc) 
    // and the value is the set of possible locations where Mr X could be
    // this will be updated whenever we change the number of moves
    private Map<Integer, Set<String>> moves;

    // the number of moves we are tracking for Mr X (should be between 0 and 5)
    private int numMoves=0;
    // What is the starting location of Mr X?
    private String startNode=null;
    
    // should we pay attention to the type of transportation Mr X might use?
    // this will be set to true and false by an ActionListener
    private boolean useTransportTypes=false;
    
    // should we color all possible moves, or just the most recent ones?
    // this will be set to true and false by an ActionListener
    private boolean inclusive=false;
    
    
    // the type of transportation to be taken in the next 5 moves
    // this will correctly be updated for you by an ActionListener
    private List<String> transportTypes = Arrays.asList("any", "any", "any", "any", "any");
    
    
    public ScotlandYardFrame() throws IOException {
        // read the image
        final Image img=ImageIO.read(new File("files/sybig.png"));
        // read the map of locations to points
        pointMap=SYSolver.readPositionPoints("files/scotpos.txt");
        // read the graph
        graph=SYSolver.readGraphFromFile(new FileInputStream("files/scotmap.txt"));
        
        canvas=new JPanel() {
            private static final long serialVersionUID = 1L;
            
            @Override
            public void paint(Graphics graphics) {
                Graphics2D g=(Graphics2D)graphics;
                // first draw the map
                g.drawImage(img, 0, 0, null);
                
                if (startNode != null) {
                    if (useTransportTypes){
                        // compute the next 5 possible moves paying attention to the transport types
                        moves=SYSolver.getNextFivePossibleMoves(graph, startNode, transportTypes);
                    } else {
                        // compute next 5 possible movies not paying attention to the transport types
                        moves=SYSolver.getNextFivePossibleMoves(graph, startNode);
                    }
                }
                
                if (moves==null || moves.size()==0) {
                    return;
                }
                
                // TODO: draw the node that has been clicked
            }
        };
        // figuring out the right sequence of these next few method calls was actually difficult
        // and I can't remember which stackoverflow articles helped me figure it out
        canvas.setPreferredSize(new Dimension(img.getWidth(null), img.getHeight(null)));
        
        //System.out.printf("width=%d, height=%d\n", img.getWidth(null), img.getHeight(null));
        this.getContentPane().add(canvas, BorderLayout.CENTER);
        this.setResizable(false);
        this.pack();
        this.setLocation(100,100);
        
        addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                System.exit(0);
            }
        });
        
        addMenu();
        
        addMouseHandlers();

    }
    
    private void addMenu() {
        JMenuBar menuBar=new JMenuBar();
        JMenu file=new JMenu("File");
        JMenuItem exit=new JMenuItem("Exit");
        exit.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                System.exit(0);
            }
        });
        file.add(exit);
        menuBar.add(file);

        // simple label for number of moves
        menuBar.add(new JLabel("# moves"));
        
        String[] numMovesString = new String[] {"0", "1", "2", "3", "4", "5"};
        JComboBox<String> numMovesSelector = new JComboBox<String>(numMovesString);
        // set max width of JComboBox
        // http://stackoverflow.com/questions/4629015/jcombobox-width
        numMovesSelector.setMaximumSize( numMovesSelector.getPreferredSize() );
        numMovesSelector.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                // ignore that it's an unchecked cast
                @SuppressWarnings("unchecked")
                JComboBox<String> source = (JComboBox<String>)e.getSource();
                // this figures out which numbered item it is in the combobox
                // and then sets the instance variable numMoves to that number
                numMoves=Integer.parseInt((String)source.getSelectedItem());
                System.out.println("num moves is now "+numMoves);
                canvas.repaint();
            }
        });
        menuBar.add(numMovesSelector);
        
        // checkbox to decide whether to show all moves along the way, or just
        // the most recent possibilities
        final JCheckBoxMenuItem inclusiveCheckBox=new JCheckBoxMenuItem("show all moves");
        inclusiveCheckBox.setSelected(false);
        menuBar.add(inclusiveCheckBox);
        inclusiveCheckBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                System.out.println(e.getActionCommand());
                inclusive=inclusiveCheckBox.isSelected();
                canvas.repaint();
            }
        });
        inclusiveCheckBox.setMaximumSize(inclusiveCheckBox.getPreferredSize());
        
        // Should we pay attention to the transportation types (taxi, bus, underground)
        // that Mr X can use when we compute the next 5 moves?
        final JCheckBoxMenuItem useTransportType=new JCheckBoxMenuItem("use transport types");
        useTransportType.setSelected(false);
        menuBar.add(useTransportType);
        useTransportType.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                System.out.println(e.getActionCommand());
                useTransportTypes=useTransportType.isSelected();
                canvas.repaint();
            }
        });
        useTransportType.setMaximumSize(useTransportType.getPreferredSize());

        // menu for selecting transportation types
        // this is actually a top-level menu with a series of sub-menus
        JMenu transportMenu = new JMenu("Transportation types");
        // Add transportation types for up to 5 moves
        transportMenu.add(makeTransportSelectorMenu());
        transportMenu.add(makeTransportSelectorMenu());
        transportMenu.add(makeTransportSelectorMenu());
        transportMenu.add(makeTransportSelectorMenu());
        transportMenu.add(makeTransportSelectorMenu());
        menuBar.add(transportMenu);
        
        // a label we can update
        menuBar.add(label);
        
        setJMenuBar(menuBar);
    }
    
    // Helper method to make a menu with 4 options (any, taxi, bus, underground)
    // that will update the menu title when you select one
    private JMenu makeTransportSelectorMenu() {
        // defaut is "any"
        JMenu selectorMenu = new JMenu("any");
        // add menu items for the 4 transport types
        selectorMenu.add(new JMenuItem("any"));
        selectorMenu.add(new JMenuItem("taxi"));
        selectorMenu.add(new JMenuItem("bus"));
        selectorMenu.add(new JMenuItem("underground"));
        // now add an ActionListener that changes the name of the 
        // transport type in the JMenu and also updates the instance variable
        // that lists the transport types
        for (int i=0; i<selectorMenu.getItemCount(); i++) {
            selectorMenu.getItem(i).addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    // I'm not going to lie: this method is kind of a hack...
                    // But, I couldn't figure out how else to make it work.
                    
                    // transportType will be any, taxi, bus, or underground
                    String transportType = e.getActionCommand();
                    // get the JMenuItem that was clicked
                    JMenuItem menuItem = (JMenuItem)e.getSource();
                    // really indirect way to get back to the menu that this menu item was part of
                    JMenu menu = (JMenu)((JPopupMenu)menuItem.getParent()).getInvoker();
                    // change the label of the menu to reflect the transportation type that was chosen
                    menu.setText(transportType);
                    // now get the top-level menu that owns the sub-menus for each move
                    JMenu topLevelMenu = (JMenu)((JPopupMenu)menu.getParent()).getInvoker();
                    // This is a relatively indirect way to figure out which menu
                    // contained the menu item that was clicked. But it works!
                    for (int i=0; i<topLevelMenu.getItemCount(); i++){
                        if (topLevelMenu.getItem(i) == menu){
                            // we've found the menu 
                            transportTypes.set(i, transportType);
                            System.out.printf("move number %d uses transport type %s\n", i, transportType);
                            break;
                        }
                    }
                }
            });
        }
        return selectorMenu;
    }
    
    private void addMouseHandlers() {
        canvas.addMouseListener(new MouseAdapter() {
            int num=1;
            @Override
            public void mouseClicked(MouseEvent e) {
                Point clicked=e.getPoint();
                System.out.printf("clicked map at (%.1f, %.1f)\n", clicked.getX(), clicked.getY());
                    
                // TODO: set the instance variable startNode to the closest point
                // to where you just clicked
                
                // Finally, redraw 
                canvas.repaint();
            }
            
            // this was for debugging purposes
            public void mouseClicked2(MouseEvent e) {
                Point p=e.getPoint();
                System.out.printf("%d %d %d\n",num, (int)p.x, (int)p.y);
                num++;
            }
        });
    }
    
    public static void main(String[] args) throws Exception {
        ScotlandYardFrame frame=new ScotlandYardFrame();
        frame.setVisible(true);
    }
}
