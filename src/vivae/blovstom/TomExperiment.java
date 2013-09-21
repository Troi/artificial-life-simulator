/**
 * This is VIVAE (Visual Vector Agent Environment)
 * a library allowing for simulations of agents in co-evolution
 * written as a bachelor project
 * by Petr Smejkal
 * at Czech Technical University in Prague
 * in 2008
 */
package vivae.blovstom;

import common.RND;
import java.awt.Dimension;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.KeyStroke;
import vivae.arena.parts.Active;
import vivae.util.FrictionBuffer;

public class TomExperiment {

    TomArena arena = null;
    JFrame f = null;
    Vector<Active> agents = null;
    private static final Dimension OBRAZOVKA = Toolkit.getDefaultToolkit().getScreenSize();

    public void createArena() {

        f = new JFrame("TomExperiment");
        arena = new TomArena(f);
        arena.loadEmptyArena(OBRAZOVKA.width - 6, OBRAZOVKA.height - 49);
        //arena.loadScenario(svgFilename);
        arena.setAllArenaPartsAntialiased(true);
        f.setBounds(0, 0, OBRAZOVKA.width, OBRAZOVKA.height);
        f.setResizable(false);
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        JMenuBar menu = new JMenuBar();
        f.setJMenuBar(menu);

        JMenu menuPause = new JMenu("Pauza");
        JMenuItem menuStartPause = new JMenuItem("Pozastavit");
        JMenuItem menuStopPause = new JMenuItem("Spustit");
//            JMenuItem menuGameLoad = new JMenuItem("Nahraj Hru");
//            JMenuItem menuGameExit = new JMenuItem("Konec...");

        menu.add(menuPause);
        menuPause.add(menuStartPause);
        menuPause.add(menuStopPause);
//            menuGame.add(menuGameLoad);
//            menuGame.add(menuGameExit);

        menuStartPause.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                arena.pause = true;
            }
        });
        menuStopPause.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                arena.pause = false;
            }
        });

        menuStartPause.setAccelerator(KeyStroke.getKeyStroke('p'));
        menuStopPause.setAccelerator(KeyStroke.getKeyStroke('P'));

        JMenu menuColor = new JMenu("Typ Vizualizace");
        JMenuItem menuColorNo = new JMenuItem("Nic");
        JMenuItem menuColorDiet = new JMenuItem("Dieta");
        JMenuItem menuColorAge = new JMenuItem("Věk");
        JMenuItem menuColorGeneration = new JMenuItem("Generace");
        JMenuItem menuColorStats = new JMenuItem("Atributy");
        JMenuItem menuColorGenom = new JMenuItem("Genom");
        JMenuItem menuColorSpiece = new JMenuItem("Druh");
        JMenuItem menuColorEnergy = new JMenuItem("Energie");

        menu.add(menuColor);
        menuColor.add(menuColorNo);
        menuColor.add(menuColorDiet);
        menuColor.add(menuColorAge);
        menuColor.add(menuColorGeneration);
        menuColor.add(menuColorStats);
        menuColor.add(menuColorGenom);
        menuColor.add(menuColorSpiece);
        menuColor.add(menuColorEnergy);



        menuColorNo.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 0;
            }
        });
        menuColorDiet.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 1;
            }
        });
        menuColorAge.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 2;
            }
        });
        menuColorGeneration.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 3;
            }
        });
        menuColorStats.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 4;
            }
        });
        menuColorGenom.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 5;
            }
        });
        menuColorSpiece.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 6;
            }
        });
        menuColorEnergy.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                TomBotStats.colorDisplaied = 7;
            }
        });


        menuColorNo.setAccelerator(KeyStroke.getKeyStroke('q'));
        menuColorDiet.setAccelerator(KeyStroke.getKeyStroke('w'));
        menuColorAge.setAccelerator(KeyStroke.getKeyStroke('e'));
        menuColorGeneration.setAccelerator(KeyStroke.getKeyStroke('r'));
        menuColorStats.setAccelerator(KeyStroke.getKeyStroke('t'));
        menuColorGenom.setAccelerator(KeyStroke.getKeyStroke('z'));
        menuColorSpiece.setAccelerator(KeyStroke.getKeyStroke('u'));
        menuColorEnergy.setAccelerator(KeyStroke.getKeyStroke('i'));

        f.getContentPane().add(arena);
        f.setVisible(true);
        arena.isVisible = true;

        arena.frictionBuffer = new FrictionBuffer(arena);
        agents = arena.getActives();
    }

    public void startExperiment() {
        arena.start();
    }

    public TomArena getArena() {
        return arena;
    }

    public static void main(String[] args) {

        RND.initialize();

        TomExperiment exp = new TomExperiment();

        exp.createArena();
        exp.startExperiment();

    }
}
