import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesCifComponent } from './preliminaries-cif.component';

describe('PreliminariesCifComponent', () => {
  let component: PreliminariesCifComponent;
  let fixture: ComponentFixture<PreliminariesCifComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesCifComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesCifComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
