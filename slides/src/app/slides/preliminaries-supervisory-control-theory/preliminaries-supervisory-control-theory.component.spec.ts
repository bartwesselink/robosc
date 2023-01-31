import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesSupervisoryControlTheoryComponent } from './preliminaries-supervisory-control-theory.component';

describe('PreliminariesSupervisoryControlTheoryComponent', () => {
  let component: PreliminariesSupervisoryControlTheoryComponent;
  let fixture: ComponentFixture<PreliminariesSupervisoryControlTheoryComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesSupervisoryControlTheoryComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesSupervisoryControlTheoryComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
